package com.acmerobotics.roadrunner.profile

import com.acmerobotics.roadrunner.util.MathUtil.solveCubic
import kotlin.math.sign

/**
 * Kinematic state of a displacement profile at any given displacement.
 */
class DisplacementState @JvmOverloads constructor(val v: Double, val a: Double = 0.0, val j: Double = 0.0) {

    /**
     * Returns the [DisplacementState] at displacement [dx].
     *
     * @param dx the change in position to query
     */
    operator fun get(dx: Double): DisplacementState {
        val t = getTime(dx)
        return DisplacementState(
                v + a * t + j / 2 * t * t,
                a + j * t,
                j
        )
    }

    /**
     * Returns the time after [dx] based on the initial parameters
     *
     * @param dx the change in position to query
     */
    fun getTime(dx: Double): Double {
        // solving the cubic 1/6jt^3 + 1/2at^2 + vt + x0 = x
        val roots = solveCubic(j / 6.0, a / 2.0, v, -dx)
        // We want the nearest root with correct the sign (assuming an increasing displacement and time)
        return roots.filter { sign(it) == sign(dx) }.sorted().getOrElse(0) { roots[0] }
    }

    override fun toString() = String.format("(v=%.3f, a=%.3f, j=%.3f)", v, a, j)
}
