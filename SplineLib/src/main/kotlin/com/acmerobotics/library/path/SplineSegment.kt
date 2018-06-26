package com.acmerobotics.library.path

import com.acmerobotics.library.Vector2d
import com.acmerobotics.library.Waypoint
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils
import kotlin.math.sqrt

class SplineSegment(start: Waypoint, end: Waypoint) : Path() {
    private val ax: Double
    private val bx: Double
    private val cx: Double
    private val dx: Double
    private val ex: Double
    private val fx: Double

    private val ay: Double
    private val by: Double
    private val cy: Double
    private val dy: Double
    private val ey: Double
    private val fy: Double

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
        val coeffX = solver.solve(targetX)
        val coeffY = solver.solve(targetY)

        fx = coeffX.getEntry(0, 0)
        ex = coeffX.getEntry(1, 0)
        dx = coeffX.getEntry(2, 0)
        cx = coeffX.getEntry(3, 0)
        bx = coeffX.getEntry(4, 0)
        ax = coeffX.getEntry(5, 0)

        fy = coeffY.getEntry(0, 0)
        ey = coeffY.getEntry(1, 0)
        dy = coeffY.getEntry(2, 0)
        cy = coeffY.getEntry(3, 0)
        by = coeffY.getEntry(4, 0)
        ay = coeffY.getEntry(5, 0)
    }

    private fun internalGet(t: Double): Vector2d {
        val x = (ax*t + bx) * (t*t*t*t) + cx * (t*t*t) + dx * (t*t) + ex * t + fx
        val y = (ay*t + by) * (t*t*t*t) + cy * (t*t*t) + dy * (t*t) + ey * t + fy
        return Vector2d(x, y)
    }

    private fun internalDeriv(t: Double): Vector2d {
        val x = (5*ax*t + 4*bx) * (t*t*t) + (3*cx*t + 2*dx) * t + ex
        val y = (5*ay*t + 4*by) * (t*t*t) + (3*cy*t + 2*dy) * t + ey
        return Vector2d(x, y)
    }

    private fun internalSecondDeriv(t: Double): Vector2d {
        val x = (20*ax*t + 12*bx) * (t*t) + 6*cx * t + 2*dx
        val y = (20*ay*t + 12*by) * (t*t) + 6*cy * t + 2*dy
        return Vector2d(x, y)
    }

    private fun internalThirdDeriv(t: Double): Vector2d {
        val x = (60*ax*t + 24*bx) * t + 6*cx
        val y = (60*ay*t + 24*by) * t + 6*cy
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

    override fun deriv(displacement: Double) = internalDeriv(displacement / length) / length

    override fun secondDeriv(displacement: Double) = internalSecondDeriv(displacement / length) / (length * length)

    override fun thirdDeriv(displacement: Double) = internalThirdDeriv(displacement / length) / (length * length * length)
}