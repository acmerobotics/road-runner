package com.acmerobotics.library

import kotlin.math.abs
import kotlin.math.sqrt

class MotionState(val x: Double, val v: Double, val a: Double) {
    operator fun get(t: Double) = MotionState(x + v * t + 0.5 * a * t * t, v + a * t, a)

    fun afterDisplacement(dx: Double): MotionState {
        val discriminant = v * v + 2 * a * dx
        return if (abs(discriminant) < MotionProfileGenerator.EPSILON) {
            MotionState(x + dx, 0.0, a)
        } else {
            MotionState(x + dx, sqrt(discriminant), a)
        }
    }

    override fun toString() = String.format("x=%.3f, v=%.3f, a=%.3f", x, v, a)
}