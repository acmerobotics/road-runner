package com.acmerobotics.library

class MotionProfile(private val segments: List<MotionSegment>) {
    operator fun get(t: Double): MotionState {
        var remainingTime = t
        for (segment in segments) {
            if (remainingTime <= segment.dt) {
                return segment[remainingTime]
            }
            remainingTime -= segment.dt
        }
        throw RuntimeException() // TODO
    }

    fun duration() = segments.map { it.dt }.sum()

    fun reversed() = MotionProfile(segments.map { it.reversed() }.reversed())
}