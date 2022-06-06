package com.acmerobotics.roadrunner

import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

object Time

fun profile(
    length: Double,
    minVel: Double,
    maxVel: (Double) -> Double,
    minMaxAccel: (Double) -> Interval,
    resolution: Double,
): MotionProfile {
    require(length > 0.0)
    require(resolution > 0.0)
    require(minVel >= 0.0)

    val samples = max(1, ceil(length / resolution).toInt())

    val velDisps = rangeMiddle(0.0, length, samples)
    val maxVels = velDisps.map(maxVel)
    val (minAccels, maxAccels) = velDisps.map { minMaxAccel(it).pair() }.unzip()

    val disps = range(0.0, length, samples + 1)
    return merge(
        forwardProfile(disps, minVel, maxVels, maxAccels),
        backwardProfile(disps, maxVels, minVel, minAccels),
    )
}

fun forwardProfile(
        length: Double,
        beginVel: Double,
        maxVel: (Double) -> Double,
        maxAccel: (Double) -> Double,
        resolution: Double,
): MotionProfile {
    val samples = max(1, ceil(length / resolution).toInt())

    val disps = rangeMiddle(0.0, length, samples)
    val maxVels = disps.map(maxVel)
    val maxAccels = disps.map(maxAccel)
    return forwardProfile(
        range(0.0, length, samples + 1),
        beginVel, maxVels, maxAccels)
}

fun backwardProfile(
    length: Double,
    maxVel: (Double) -> Double,
    endVel: Double,
    minAccel: (Double) -> Double,
    resolution: Double,
): MotionProfile {
    val samples = max(1, ceil(length / resolution).toInt())

    val disps = rangeMiddle(0.0, length, samples)
    val maxVels = disps.map(maxVel)
    val minAccels = disps.map(minAccel)
    return backwardProfile(
        range(0.0, length, samples + 1),
        maxVels, endVel, minAccels)
}

data class MotionProfile(
    val disps: List<Double>,
    val vels: List<Double>,
    val accels: List<Double>,
) {
    init {
        require(disps.size == vels.size)
        require(disps.size == accels.size + 1)
    }
}

fun merge(p1: MotionProfile, p2: MotionProfile): MotionProfile {
    val disps = mutableListOf(0.0)
    val vels = mutableListOf(min(p1.vels[0], p2.vels[0]))
    val accels = mutableListOf<Double>()

    var lastMin1 = p1.vels[0] < p2.vels[0]

    var i = 1; var j = 1
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
                    sqrt(p2.vels[j] * p2.vels[j] +
                            2 * accel2 * (p1.disps[i - 1] - endDisp))
                )
            } else {
                j++
                Pair(
                    sqrt(p1.vels[i] * p1.vels[i] +
                            2 * accel1 * (p2.disps[j - 1] - endDisp)),
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

    return MotionProfile(disps, vels, accels)
}

// TODO: doesn't enforce invariants => private
// maxVels, maxAccels are sampled in the *middle*
// TODO: we should have displacement samples to avoid floating point stuff
@Suppress("NAME_SHADOWING")
private fun forwardProfile(
    disps: List<Double>,
    beginVel: Double,
    maxVels: List<Double>,
    maxAccels: List<Double>,
): MotionProfile {
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

    return MotionProfile(newDisps, vels, accels)
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
        MotionProfile(
            it.disps.map { x -> it.disps.last() - x }.reversed(),
            it.vels.reversed(),
            it.accels.reversed().map { a -> -a },
        )
    }

