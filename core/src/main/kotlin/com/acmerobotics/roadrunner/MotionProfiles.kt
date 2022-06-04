package com.acmerobotics.roadrunner

import kotlin.math.ceil
import kotlin.math.max

object Time

data class Interval(val min: Double, val max: Double)

// TODO: List<Double> vs DoubleArray
// What's the right line of non-pessimization?
fun range(begin: Double, end: Double, samples: Int): List<Double> {
    require(samples >= 1)
    return when (samples) {
        1 -> listOf(begin)
        else -> (0 until samples).map { begin + (end - begin) * it / (samples - 1).toDouble() }
    }
}

// TODO: what does a safe API look like?
fun profile(
    length: Double,
    // TODO: do we need to separate these functions?
    maxVel: (Double) -> Double,
    minMaxAccel: (Double) -> Interval,
    resolution: Double,
    minVel: Double,
): MotionProfile {
    require(length >= 0.0)
    require(minVel >= 0.0)
    require(resolution > 0.0)

    val samples = max(2, ceil(length / resolution).toInt())

    val disps = range(0.0, length, samples)

    val maxVels = disps.map {
        val v = maxVel(it)
        require(v > 0.0)
        max(minVel, v)
    }

    val (minAccels, maxAccels) = disps.map {
        val (amin, amax) = minMaxAccel(it)
        require(amin < 0.0)
        require(amax > 0.0)
        Pair(amin, amax)
    }.unzip()

    val p1 = forwardProfile(maxVels, maxAccels, minVel)
    val p2 = backwardProfile(maxVels, minAccels.map { -it }, minVel)

    // TODO: combine
    val p = listOf<ProfileSegment>()
    return p
}

fun forwardProfile(
        length: Double,
        maxVel: (Double) -> Double,
        maxAccel: (Double) -> Double,
        minVel: Double,
        resolution: Double,
): MotionProfile {
    TODO()
}

class ProfileSegment {
    // TODO
    fun reverse() = this
}

typealias MotionProfile = List<ProfileSegment>

fun forwardProfile(
        maxVels: List<Double>,
        maxAccels: List<Double>,
        beginVel: Double,
): MotionProfile {
    return listOf()
}

fun backwardProfile(
        maxVels: List<Double>,
        maxAccels: List<Double>,
        endVel: Double,
): MotionProfile {
    return forwardProfile(
            maxVels.reversed(), maxAccels.reversed(), endVel
    )
            .map { it.reverse() }
            .reversed()
}
