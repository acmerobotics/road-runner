package com.acmerobotics.splinelib.profile

import kotlin.math.max
import kotlin.math.min

class SegmentedMotionProfile(private val segments: List<MotionSegment>) : MotionProfile() {
    override operator fun get(t: Double): MotionState {
        var remainingTime = max(0.0, min(t, duration()))
        for (segment in segments) {
            if (remainingTime <= segment.dt) {
                return segment[remainingTime]
            }
            remainingTime -= segment.dt
        }
        return segments.last().end()
    }

    override fun duration() = segments.map { it.dt }.sum()

    fun reversed() = SegmentedMotionProfile(segments.map { it.reversed() }.reversed())
}