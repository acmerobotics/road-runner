package com.acmerobotics.roadrunner.profile

import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Kinematic state of a motion profile at any given time.
 */
class MotionState(val x: Double, val v: Double, val a: Double, val j: Double = 0.0) {

    /**
     * Returns the [MotionState] at time [t].
     */
    operator fun get(t: Double) =
        MotionState(
                x + v * t + a / 2 * t * t + j / 6 * t * t * t,
                v + a * t + j / 2 * t * t,
                a + j * t,
                j
        )

    /**
     * Returns the [MotionState] after displacement [dx].
     */
    @Deprecated("this is no longer valid for non-zero jerk")
    fun afterDisplacement(dx: Double): MotionState {
        val discriminant = v * v + 2 * a * dx
        return if (abs(discriminant) < 1e-6) {
            MotionState(x + dx, 0.0, a)
        } else {
            MotionState(x + dx, sqrt(discriminant), a)
        }
    }

    override fun toString() = String.format("(x=%.3f, v=%.3f, a=%.3f, j=%.3f)", x, v, a, j)
}