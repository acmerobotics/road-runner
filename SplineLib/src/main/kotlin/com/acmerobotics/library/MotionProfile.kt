package com.acmerobotics.library

class MotionProfile(private val segments: List<MotionSegment>, private val reversed: Boolean = false) {
    operator fun get(t: Double): MotionState {
        var remainingTime = if (reversed) duration() - t else t
        for (segment in segments) {
            if (remainingTime <= segment.dt) {
                return segment[remainingTime]
            }
            remainingTime -= segment.dt
        }
        return segments.last().end()
    }

    fun duration() = segments.map { it.dt }.sum()

    fun reversed() = MotionProfile(segments, !reversed)
}