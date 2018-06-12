package com.acmerobotics.library

class MotionSegment(val start: MotionState, val dt: Double) {
    operator fun get(t: Double) = start[t]
    fun end() = start[dt]
}