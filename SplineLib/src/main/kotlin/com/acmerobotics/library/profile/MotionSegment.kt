package com.acmerobotics.library.profile

class MotionSegment(val start: MotionState, val dt: Double) {
    operator fun get(t: Double) = start[t]
    fun end() = start[dt]
    fun reversed(): MotionSegment {
        val end = end()
        val state = MotionState(end.x, -end.v, end.a)
        return MotionSegment(state, dt)
    }
}