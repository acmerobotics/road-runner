package com.acmerobotics.roadrunner.profile

/**
 * Segment of a motion profile with constant acceleration.
 *
 * @param start start motion state
 * @param dt time delta
 */
class MotionSegment(val start: MotionState, val dt: Double) {

    /**
     * Returns the [MotionState] at time [t].
     */
    operator fun get(t: Double) = start[t]

    /**
     * Returns the [MotionState] at the end of the segment (time [dt]).
     */
    fun end() = start[dt]

    /**
     * Returns a flipped (negated) version of the segment.
     */
    fun flipped() = MotionSegment(start.flipped(), dt)

    override fun toString() = "($start, $dt)"
}