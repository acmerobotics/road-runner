package com.acmerobotics.library

import kotlin.math.max
import kotlin.math.min

class MotionProfile(private val segments: List<MotionSegment>) {
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

    fun duration() = segments.map { it.dt }.sum()

    fun reversed() = MotionProfile(segments.map { it.reversed() }.reversed())

    fun start() = get(0.0)
    fun end() = get(duration())
}