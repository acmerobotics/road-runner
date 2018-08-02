package com.acmerobotics.splinelib.profile

import kotlin.math.max
import kotlin.math.min

/**
 * Trapezoidal motion profile composed of motion segments.
 *
 * @param segments profile motion segments
 */
class MotionProfile(private val segments: List<MotionSegment>) {

    /**
     * Returns the [MotionState] at time [t].
     */
    operator fun get(t: Double): MotionState {
        var remainingTime = max(0.0, min(t, duration()))
        for (segment in segments) {
            if (remainingTime <= segment.dt) {
                return segment[remainingTime]
            }
            remainingTime -= segment.dt
        }
        return segments.last().end()
    }

    /**
     * Returns the duration of the motion profile.
     */
    fun duration() = segments.map { it.dt }.sum()

    /**
     * Returns a reversed version of the motion profile.
     */
    fun reversed() = MotionProfile(segments.map { it.reversed() }.reversed())

    /**
     * Returns the end [MotionState].
     */
    fun end() = get(duration())
}