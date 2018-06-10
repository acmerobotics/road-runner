package com.acmerobotics.library

class MotionSegment(val start: MotionState, val dt: Double) {
    fun end() = MotionState(start.x + start.v * dt + 0.5 * start.a * dt * dt, start.v + start.a * dt, start.a)
    override fun toString() = String.format("[%s, dt=%.3f]", start.toString(), dt)
}