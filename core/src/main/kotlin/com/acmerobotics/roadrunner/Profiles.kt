@file:JvmName("Profiles")

package com.acmerobotics.roadrunner

import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

class Time

fun constantProfile(
    length: Double,
    beginEndVel: Double,
    maxVel: Double,
    minMaxAccel: Interval,
) = profile(length, beginEndVel, { maxVel }, { minMaxAccel }, length)

fun profile(
    length: Double,
    beginEndVel: Double,
    maxVel: (Double) -> Double,
    minMaxAccel: (Double) -> Interval,
    resolution: Double,
): DisplacementProfile {
    require(length > 0.0)
    require(resolution > 0.0)
    require(beginEndVel >= 0.0)

    val samples = max(1, ceil(length / resolution).toInt())

    val velDisps = rangeMiddle(0.0, length, samples)
    val maxVels = velDisps.map(maxVel)
    val (minAccels, maxAccels) = velDisps.map { minMaxAccel(it).pair() }.unzip()

    val disps = range(0.0, length, samples + 1)
    return merge(
        forwardProfile(disps, beginEndVel, maxVels, maxAccels),
        backwardProfile(disps, maxVels, beginEndVel, minAccels),
    )
}

fun forwardProfile(
    length: Double,
    beginVel: Double,
    maxVel: (Double) -> Double,
    maxAccel: (Double) -> Double,
    resolution: Double,
): DisplacementProfile {
    val samples = max(1, ceil(length / resolution).toInt())

    val disps = rangeMiddle(0.0, length, samples)
    val maxVels = disps.map(maxVel)
    val maxAccels = disps.map(maxAccel)
    return forwardProfile(
        range(0.0, length, samples + 1),
        beginVel, maxVels, maxAccels
    )
}

fun backwardProfile(
    length: Double,
    maxVel: (Double) -> Double,
    endVel: Double,
    minAccel: (Double) -> Double,
    resolution: Double,
): DisplacementProfile {
    val samples = max(1, ceil(length / resolution).toInt())

    val disps = rangeMiddle(0.0, length, samples)
    val maxVels = disps.map(maxVel)
    val minAccels = disps.map(minAccel)
    return backwardProfile(
        range(0.0, length, samples + 1),
        maxVels, endVel, minAccels
    )
}

data class DisplacementProfile(
    val disps: List<Double>,
    val vels: List<Double>,
    val accels: List<Double>,
) {
    val length = disps.last()

    init {
        require(disps.size == vels.size)
        require(disps.size == accels.size + 1)
    }

    operator fun get(x: Double): DualNum<Time> {
        val index = disps.binarySearch(x)
        return when {
            index >= disps.lastIndex ->
                DualNum(doubleArrayOf(x, vels[index], 0.0))
            index >= 0 ->
                DualNum(doubleArrayOf(x, vels[index], accels[index]))
            else -> {
                val insIndex = -(index + 1)
                when {
                    insIndex <= 0 ->
                        DualNum(doubleArrayOf(0.0, 0.0, 0.0))
                    insIndex >= disps.size ->
                        DualNum(doubleArrayOf(length, 0.0, 0.0))
                    else -> {
                        val dx = x - disps[insIndex - 1]
                        val v0 = vels[insIndex - 1]
                        val a = accels[insIndex - 1]

                        DualNum(
                            doubleArrayOf(
                                x,
                                sqrt(v0 * v0 + 2 * a * dx),
                                a
                            )
                        )
                    }
                }
            }
        }
    }
}

fun timeScan(p: DisplacementProfile): List<Double> {
    val times = mutableListOf(0.0)
    for (i in p.accels.indices) {
        times.add(
            times.last() +
                if (p.accels[i] == 0.0) {
                    (p.disps[i + 1] - p.disps[i]) / p.vels[i]
                } else {
                    (p.vels[i + 1] - p.vels[i]) / p.accels[i]
                }
        )
    }
    return times
}

data class TimeProfile(
    val dispProfile: DisplacementProfile,
    val times: List<Double>,
) {
    val duration = times.last()

    constructor(dispProfile: DisplacementProfile) : this(dispProfile, timeScan(dispProfile))

    init {
        require(times.size == dispProfile.disps.size)
    }

    operator fun get(t: Double): DualNum<Time> {
        val index = times.binarySearch(t)
        return when {
            index >= times.lastIndex ->
                DualNum(
                    doubleArrayOf(
                        dispProfile.disps[index], dispProfile.vels[index], 0.0
                    )
                )
            index >= 0 ->
                DualNum(
                    doubleArrayOf(
                        dispProfile.disps[index], dispProfile.vels[index], dispProfile.accels[index]
                    )
                )
            else -> {
                val insIndex = -(index + 1)
                when {
                    insIndex <= 0 ->
                        DualNum(doubleArrayOf(0.0, 0.0, 0.0))
                    insIndex >= times.size ->
                        DualNum(doubleArrayOf(dispProfile.length, 0.0, 0.0))
                    else -> {
                        val dt = t - times[insIndex - 1]
                        val x0 = dispProfile.disps[insIndex - 1]
                        val v0 = dispProfile.vels[insIndex - 1]
                        val a = dispProfile.accels[insIndex - 1]

                        DualNum(
                            doubleArrayOf(
                                (0.5 * a * dt + v0) * dt + x0,
                                a * dt + v0,
                                a
                            )
                        )
                    }
                }
            }
        }
    }

    fun getByPos(x: Double) = dispProfile[x]
}

fun merge(p1: DisplacementProfile, p2: DisplacementProfile): DisplacementProfile {
    val disps = mutableListOf(0.0)
    val vels = mutableListOf(min(p1.vels[0], p2.vels[0]))
    val accels = mutableListOf<Double>()

    var lastMin1 = p1.vels[0] < p2.vels[0]

    var i = 1
    var j = 1
    while (i < p1.disps.size && j < p2.disps.size) {
        val endDisp = min(p1.disps[i], p2.disps[j])
        val accel1 = p1.accels[i - 1]
        val accel2 = p2.accels[j - 1]

        val (endVel1, endVel2) =
            if (p1.disps[i] == p2.disps[j]) {
                i++
                j++
                Pair(
                    p1.vels[i - 1],
                    p2.vels[j - 1],
                )
            } else if (p1.disps[i] < p2.disps[j]) {
                i++
                Pair(
                    p1.vels[i - 1],
                    sqrt(
                        p2.vels[j] * p2.vels[j] +
                            2 * accel2 * (p1.disps[i - 1] - endDisp)
                    )
                )
            } else {
                j++
                Pair(
                    sqrt(
                        p1.vels[i] * p1.vels[i] +
                            2 * accel1 * (p2.disps[j - 1] - endDisp)
                    ),
                    p2.vels[j - 1]
                )
            }

        val min1 = endVel1 < endVel2
        if (min1 == lastMin1) {
            disps.add(endDisp)
            if (min1) {
                vels.add(endVel1)
                accels.add(accel1)
            } else {
                vels.add(endVel2)
                accels.add(accel2)
            }
        } else if (accel1 == accel2) {
            disps.add(endDisp)
            vels.add(endVel1)
            accels.add(0.0)
        } else {
            val dx = (endVel2 * endVel2 - endVel1 * endVel1) / (2 * (accel2 - accel1))
            disps.add(endDisp - dx)
            vels.add(sqrt(endVel1 * endVel1 - 2 * accel1 * dx))
            accels.add(max(accel1, accel2))

            disps.add(endDisp)
            vels.add(min(endVel1, endVel2))
            accels.add(min(accel1, accel2))
        }

        lastMin1 = min1
    }

    return DisplacementProfile(disps, vels, accels)
}

// maxVels, maxAccels are sampled in the *middle*
@Suppress("NAME_SHADOWING")
private fun forwardProfile(
    disps: List<Double>,
    beginVel: Double,
    maxVels: List<Double>,
    maxAccels: List<Double>,
): DisplacementProfile {
    val newDisps = mutableListOf(0.0)
    val vels = mutableListOf(beginVel)
    val accels = mutableListOf<Double>()

    maxVels
        .zip(maxAccels)
        .zip(disps.drop(1))
        .fold(disps[0]) { beginDisp, (c, endDisp) ->
            val (maxVel, maxAccel) = c

            val beginVel = vels.last()
            if (beginVel >= maxVel) {
                newDisps.add(endDisp)
                vels.add(maxVel)
                accels.add(0.0)
            } else {
                val endVel = sqrt(beginVel * beginVel + 2 * maxAccel * (endDisp - beginDisp))
                if (endVel <= maxVel) {
                    newDisps.add(endDisp)
                    vels.add(endVel)
                    accels.add(maxAccel)
                } else {
                    val accelDx = (maxVel * maxVel - beginVel * beginVel) / (2 * maxAccel)

                    newDisps.add(beginDisp + accelDx)
                    vels.add(maxVel)
                    accels.add(maxAccel)

                    newDisps.add(endDisp)
                    vels.add(maxVel)
                    accels.add(0.0)
                }
            }

            endDisp
        }

    return DisplacementProfile(newDisps, vels, accels)
}

// maxVels, minAccels are sampled in the *middle*
private fun backwardProfile(
    disps: List<Double>,
    maxVels: List<Double>,
    endVel: Double,
    minAccels: List<Double>,
) = forwardProfile(
    disps, endVel, maxVels.reversed(), minAccels.reversed().map { -it }
).let {
    DisplacementProfile(
        it.disps.map { x -> it.length - x }.reversed(),
        it.vels.reversed(),
        it.accels.reversed().map { a -> -a },
    )
}
