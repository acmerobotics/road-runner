package com.acmerobotics.splinelib.profile

abstract class MotionProfile {
    abstract operator fun get(t: Double): MotionState

    abstract fun duration(): Double

    fun start() = get(0.0)

    fun end() = get(duration())
}