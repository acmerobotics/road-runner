package com.acmerobotics.roadrunner.util

import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Various math utilities.
 */
object MathUtil {

    /**
     * Returns the real solutions to the quadratic ax^2 + bx + c.
     */
    @JvmStatic
    fun solveQuadratic(a: Double, b: Double, c: Double): List<Double> {
        val disc = b * b - 4 * a * c
        return when {
            disc epsilonEquals 0.0 -> listOf(-b / (2 * a))
            disc > 0.0 -> listOf(
                    (-b + sqrt(disc)) / (2 * a),
                    (-b - sqrt(disc)) / (2 * a)
            )
            else -> emptyList()
        }
    }

    /**
     * Numerically compute dy/dx from the given x and y values. The returned list is padded to match
     * the length of the original sequences.
     * @param x x-values
     * @param y y-values
     * @return derivative values
     */
    fun numericalDerivative(x: List<Double>, y: List<Double>): List<Double> {
        val deriv = mutableListOf<Double>()
        for (i in 2 until x.size) {
            deriv.add((y[i] - y[i - 2]) / (x[i] - x[i - 2]))
        }
        deriv.add(0, deriv[0])
        deriv.add(deriv[deriv.size - 1])
        return deriv
    }
}

const val EPSILON = 1e-6

infix fun Double.epsilonEquals(other: Double) = abs(this - other) < EPSILON
