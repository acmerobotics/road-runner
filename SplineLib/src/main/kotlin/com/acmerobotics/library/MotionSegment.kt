package com.acmerobotics.library

class MotionSegment(val start: MotionState, val dt: Double) {
    operator fun get(t: Double) = start[t]
    fun end() = start[dt]
    override fun toString() = String.format("[%s, dt=%.3f]", start.toString(), dt)
}