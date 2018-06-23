package com.acmerobotics.library

import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils
import org.apache.commons.math3.linear.RealMatrix
import kotlin.math.sqrt

class SplineSegment(val start: Waypoint, val end: Waypoint) : Path() {
    private val coeffX: RealMatrix
    private val coeffY: RealMatrix

    companion object {
        private val COEFF_MATRIX = MatrixUtils.createRealMatrix(
            arrayOf(
                doubleArrayOf(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                doubleArrayOf(0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
                doubleArrayOf(0.0, 0.0, 2.0, 0.0, 0.0, 0.0),
                doubleArrayOf(1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
                doubleArrayOf(0.0, 1.0, 2.0, 3.0, 4.0, 5.0),
                doubleArrayOf(0.0, 0.0, 2.0, 6.0, 12.0, 20.0)
            )
        )
        private const val LENGTH_SAMPLES = 1000
    }

    init {
        val targetX =
            MatrixUtils.createRealMatrix(arrayOf(doubleArrayOf(start.x, start.dx, start.dx2, end.x, end.dx, end.dx2)))
                .transpose()
        val targetY =
            MatrixUtils.createRealMatrix(arrayOf(doubleArrayOf(start.y, start.dy, start.dy2, end.y, end.dy, end.dy2)))
                .transpose()

        val solver = LUDecomposition(COEFF_MATRIX).solver
        coeffX = solver.solve(targetX)
        coeffY = solver.solve(targetY)
    }

    private fun internalGet(t: Double): Vector2d {
        val basis = MatrixUtils.createRealMatrix(
            arrayOf(
                doubleArrayOf(
                    1.0,
                    t,
                    t * t,
                    t * t * t,
                    t * t * t * t,
                    t * t * t * t * t
                )
            )
        )
        val x = basis.multiply(coeffX).getEntry(0, 0)
        val y = basis.multiply(coeffY).getEntry(0, 0)
        return Vector2d(x, y)
    }

    private fun internalDeriv(t: Double): Vector2d {
        val basis = MatrixUtils.createRealMatrix(
            arrayOf(
                doubleArrayOf(
                    0.0,
                    1.0,
                    2.0 * t,
                    3.0 * t * t,
                    4.0 * t * t * t,
                    5.0 * t * t * t * t
                )
            )
        )
        val x = basis.multiply(coeffX).getEntry(0, 0)
        val y = basis.multiply(coeffY).getEntry(0, 0)
        return Vector2d(x, y)
    }

    private fun internalSecondDeriv(t: Double): Vector2d {
        val basis = MatrixUtils.createRealMatrix(
            arrayOf(
                doubleArrayOf(
                    0.0,
                    0.0,
                    2.0,
                    6.0 * t,
                    12.0 * t * t,
                    20.0 * t * t * t
                )
            )
        )
        val x = basis.multiply(coeffX).getEntry(0, 0)
        val y = basis.multiply(coeffY).getEntry(0, 0)
        return Vector2d(x, y)
    }

    private fun internalThirdDeriv(t: Double): Vector2d {
        val basis = MatrixUtils.createRealMatrix(
            arrayOf(
                doubleArrayOf(
                    0.0,
                    0.0,
                    0.0,
                    6.0,
                    24.0 * t,
                    60.0 * t * t
                )
            )
        )
        val x = basis.multiply(coeffX).getEntry(0, 0)
        val y = basis.multiply(coeffY).getEntry(0, 0)
        return Vector2d(x, y)
    }

    private fun computeLength(samples: Int): Double {
        val dx = 1.0 / samples
        var sum = 0.0
        var lastIntegrand = 0.0
        for (i in 1..samples) {
            val t = i * dx
            val deriv = internalDeriv(t)
            val integrand = sqrt(deriv.x * deriv.x + deriv.y * deriv.y)
            sum += (integrand + lastIntegrand) / 2.0
            lastIntegrand = integrand
        }
        return sum * dx
    }

    private val length by lazy { computeLength(LENGTH_SAMPLES) }

    override fun length() = length

    override operator fun get(displacement: Double) = internalGet(displacement / length)

    override fun deriv(displacement: Double) = internalDeriv(displacement / length)

    override fun secondDeriv(displacement: Double) = internalSecondDeriv(displacement / length)

    override fun thirdDeriv(displacement: Double) = internalThirdDeriv(displacement / length)
}