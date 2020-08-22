package com.acmerobotics.roadrunner.util

import kotlin.math.*

/**
 * Various math utilities.
 */
object MathUtil {

    /**
     * Returns the real solutions to the quadratic [a]x^2 + [b]x + [c] = 0.
     */
    @JvmStatic
    fun solveQuadratic(a: Double, b: Double, c: Double): DoubleArray {
        if (a epsilonEquals 0.0) {
            return doubleArrayOf(-c / b)
        }

        val disc = b * b - 4 * a * c
        return when {
            disc epsilonEquals 0.0 -> doubleArrayOf(-b / (2 * a))
            disc > 0.0 -> doubleArrayOf(
                    (-b + sqrt(disc)) / (2 * a),
                    (-b - sqrt(disc)) / (2 * a)
            )
            else -> doubleArrayOf()
        }
    }

    /**
     * Similar to Java cbrt, but implemented with kotlin math (preserves sign)
     */
    @JvmStatic
    fun cbrt(x: Double) = sign(x) * abs(x).pow(1.0 / 3.0)

    /**
     * Returns real solutions to the cubic [a]x^3 + [b]x^2 + [c]x + [d] = 0
     * [Reference](https://github.com/davidzof/wattzap/blob/a3065da/src/com/wattzap/model/power/Cubic.java#L125-L182)
     */
    @JvmStatic
    fun solveCubic(a: Double, b: Double, c: Double, d: Double): DoubleArray {
        if (a epsilonEquals 0.0) {
            return solveQuadratic(b, c, d)
        }

        val a2 = b / a
        val a1 = c / a
        val a0 = d / a

        val lambda = a2 / 3.0
        val Q = (3.0 * a1 - a2 * a2) / 9.0
        val R = (9.0 * a1 * a2 - 27.0 * a0 - 2.0 * a2 * a2 * a2) / 54.0
        val Q3 = Q * Q * Q
        val R2 = R * R
        val D = Q3 + R2

        return when {
            D < 0.0 -> { // 3 unique reals
                val theta = acos(R / sqrt(-Q3))
                val sqrtQ = sqrt(-Q)
                doubleArrayOf(
                        2.0 * sqrtQ * cos(theta / 3.0) - lambda,
                        2.0 * sqrtQ * cos((theta + 2.0 * PI) / 3.0) - lambda,
                        2.0 * sqrtQ * cos((theta + 4.0 * PI) / 3.0) - lambda
                )
            }
            D > 0.0 -> { // 1 real
                val sqrtD = sqrt(D)
                val S = cbrt(R + sqrtD)
                val T = cbrt(R - sqrtD)
                doubleArrayOf(S + T - lambda)
            }
            else -> { // 2 unique (3 total) reals
                val cbrtR = cbrt(R)
                doubleArrayOf(2.0 * cbrtR - lambda, -cbrtR - lambda)
            }
        }
    }
}

const val EPSILON = 1e-6

infix fun Double.epsilonEquals(other: Double) = abs(this - other) < EPSILON
