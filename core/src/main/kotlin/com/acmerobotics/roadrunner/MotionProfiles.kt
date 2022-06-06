package com.acmerobotics.roadrunner

import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

object Time

// TODO: what does a safe API look like?
fun profile(
    length: Double,
    // TODO: do we need to separate these functions?
    maxVel: (Double) -> Double,
    minMaxAccel: (Double) -> Interval,
    resolution: Double,
    minVel: Double,
): MotionProfile {
    require(length > 0.0)
    require(resolution > 0.0)
    require(minVel >= 0.0)

    val samples = max(2, ceil(length / resolution).toInt())

    val (dx, velDisps) = range(0.0, length, samples)
    val maxVels = velDisps.map(maxVel)
    val (minAccels, maxAccels) = velDisps
        .drop(1)
        .map { minMaxAccel(it - 0.5 * dx).pair() }
        .unzip()

    return forwardProfile(length, minVel, maxVels.drop(1), maxAccels).merge(
        backwardProfile(length, maxVels.dropLast(1), minAccels, minVel))
}

fun forwardProfile(
        length: Double,
        // TODO: should this have a default?
        beginVel: Double,
        maxVel: (Double) -> Double,
        maxAccel: (Double) -> Double,
        resolution: Double,
): MotionProfile {
    val samples = max(2, ceil(length / resolution).toInt())

    val (dx, velDisps) = range(0.0, length, samples)
    return velDisps
        .drop(1)
        .let {
            val maxVels = it.map(maxVel)
            val maxAccels = it.map { x -> maxAccel(x - 0.5 * dx) }
            forwardProfile(dx, beginVel, maxVels, maxAccels)
        }
}

// TODO: naming
data class MotionProfile(
    val disps: List<Double>,
    val vels: List<Double>,
    val accels: List<Double>,
) {
    init {
        require(disps.size == vels.size)
        require(disps.size == accels.size + 1)
    }

    fun reversed() = MotionProfile(
        disps.reversed(),
        vels.reversed(),
        accels.reversed().map { -it },
    )

    fun merge(p: MotionProfile): MotionProfile {
        val disps = mutableListOf(0.0)
        val vels = mutableListOf(min(vels[0], p.vels[0]))
        val accels = mutableListOf<Double>()

        var lastMinThis = vels[0] < p.vels[0]

        // TODO: this is nasty no matter how you slice it
        var i = 1; var j = 1
        while (i < disps.size && j < p.disps.size) {
            if (disps[i] < p.disps[j]) {
                val v = sqrt(
                    p.vels[j] * p.vels[j] +
                    2 * p.accels[j - 1] * (disps[i] - p.disps[j - 1])
                )

                val minThis = vels[i] < v
                if (lastMinThis == minThis) {
                    disps.add(disps[i])
                    if (minThis) {
                        disps.add(vels[i])
                        disps.add(accels[i - 1])
                    } else {
                        disps.add(v)
                        disps.add(p.accels[j - 1])
                    }
                } else {
                    val dx = (v * v - vels[i] * vels[i]) / (2 * (p.accels[j - 1] - accels[i - 1]))
                    disps.add(disps[i] - dx)
                    vels.add(sqrt(v * v - 2 * p.accels[j - 1] * dx))
                    accels.add(max(accels[i - 1], p.accels[j - 1]))

                    disps.add(disps[i])
                    vels.add(min(vels[i], v))
                    accels.add(min(accels[i - 1], p.accels[j - 1]))
                }

                lastMinThis = minThis

                i++
            } else {
                val v = sqrt(
                    vels[i] * vels[i] +
                        2 * accels[i - 1] * (p.disps[j] - disps[i - 1])
                )

                val minThis = v < p.vels[j]
                if (lastMinThis == minThis) {
                    disps.add(p.disps[j])
                    if (minThis) {
                        disps.add(v)
                        disps.add(accels[i - 1])
                    } else {
                        disps.add(p.vels[j])
                        disps.add(p.accels[j - 1])
                    }
                } else {
                    val dx = (v * v - p.vels[j] * p.vels[j]) / (2 * (accels[i - 1] - p.accels[j - 1]))
                    disps.add(p.disps[j] - dx)
                    vels.add(sqrt(v * v - 2 * accels[i - 1] * dx))
                    accels.add(max(accels[i - 1], p.accels[j - 1]))

                    disps.add(p.disps[j])
                    vels.add(min(p.vels[j], v))
                    accels.add(min(accels[i - 1], p.accels[j - 1]))
                }

                lastMinThis = minThis

                j++
            }
        }

        return MotionProfile(disps, vels, accels)
    }
}

// TODO: doesn't enforce invariants => private
// maxVels are sampled at the *end*
// maxAccels are sampled in the *middle*
@Suppress("NAME_SHADOWING")
private fun forwardProfile(
    dx: Double,
    beginVel: Double,
    maxVels: List<Double>,
    maxAccels: List<Double>,
): MotionProfile {
    val disps = mutableListOf(0.0)
    val vels = mutableListOf(beginVel)
    val accels = mutableListOf<Double>()

    maxVels
        .zip(maxAccels)
        .forEach { (maxVel, maxAccel) ->
            val beginDisp = disps.last()
            val beginVel = vels.last()
            val endVel = sqrt(beginVel * beginVel + 2 * maxAccel * dx)
            if (endVel <= maxVel) {
                disps.add(beginDisp + dx)
                vels.add(endVel)
                accels.add(maxAccel)
            } else {
                val accelDx = (maxVel * maxVel - beginVel * beginVel) / (2 * maxAccel)
                disps.add(beginDisp + accelDx)
                vels.add(maxVel)
                accels.add(maxAccel)

                disps.add(beginDisp + dx)
                vels.add(maxVel)
                accels.add(0.0)
            }
        }

    return MotionProfile(disps, vels, accels)
}

// maxVels are sampled at the *begin*
// minAccels are sampled in the *middle*
private fun backwardProfile(
        dx: Double,
        maxVels: List<Double>,
        minAccels: List<Double>,
        endVel: Double,
) = forwardProfile(
            dx, endVel, maxVels.reversed(), minAccels.reversed().map { -it }
    ).reversed()
