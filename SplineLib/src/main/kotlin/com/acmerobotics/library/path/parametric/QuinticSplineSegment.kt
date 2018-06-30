package com.acmerobotics.library.path.parametric

import com.acmerobotics.library.InterpolatingTreeMap
import com.acmerobotics.library.Pose2d
import com.acmerobotics.library.Vector2d
import com.acmerobotics.library.Waypoint
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils
import java.lang.Math.pow
import kotlin.math.sqrt

class QuinticSplineSegment(start: Waypoint, end: Waypoint) : ParametricCurve() {
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

    private val length: Double

    private val arcLengthSamples = InterpolatingTreeMap()

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

        fun fromPoses(start: Pose2d, end: Pose2d): QuinticSplineSegment {
            val startWaypoint = Waypoint(start.x, start.y, Math.cos(start.heading), Math.sin(start.heading))
            val endWaypoint = Waypoint(end.x, end.y, Math.cos(end.heading), Math.sin(end.heading))
            return QuinticSplineSegment(startWaypoint, endWaypoint)
        }
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

        arcLengthSamples[0.0] = 0.0
        val dx = 1.0 / LENGTH_SAMPLES
        var sum = 0.0
        var lastIntegrand = 0.0
        for (i in 1..LENGTH_SAMPLES) {
            val t = i * dx
            val deriv = internalDeriv(t)
            val integrand = sqrt(deriv.x * deriv.x + deriv.y * deriv.y) * dx
            sum += (integrand + lastIntegrand) / 2.0
            lastIntegrand = integrand

            arcLengthSamples[sum] = t
        }
        length = sum
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

    override fun length() = length

    override operator fun get(displacement: Double): Vector2d {
        val t = arcLengthSamples.getInterpolated(displacement) ?: 0.0
        return internalGet(t)
    }

    override fun deriv(displacement: Double): Vector2d {
        val t = arcLengthSamples.getInterpolated(displacement) ?: 0.0
        val deriv = internalDeriv(t)
        return deriv / sqrt(deriv.x * deriv.x + deriv.y * deriv.y)
    }

    override fun secondDeriv(displacement: Double): Vector2d {
        val t = arcLengthSamples.getInterpolated(displacement) ?: 0.0
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val numerator = -(deriv.x * secondDeriv.x + deriv.y * secondDeriv.y)
        val denominator = deriv.x * deriv.x + deriv.y * deriv.y
        return secondDeriv / denominator + deriv * numerator / (denominator * denominator)
    }

    override fun thirdDeriv(displacement: Double): Vector2d {
        val t = arcLengthSamples.getInterpolated(displacement) ?: 0.0
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val thirdDeriv = internalThirdDeriv(t)
        val firstNumerator = -(deriv.x * secondDeriv.x + deriv.y * secondDeriv.y)
        val secondNumeratorFirstTerm = secondDeriv.x * secondDeriv.x + secondDeriv.y * secondDeriv.y +
                deriv.x * thirdDeriv.x + deriv.y * thirdDeriv.y
        val secondNumeratorSecondTerm = -4.0 * firstNumerator
        val denominator = deriv.x * deriv.x + deriv.y * deriv.y
        return thirdDeriv / pow(denominator, 1.5) + secondDeriv * 3.0 * firstNumerator / pow(denominator, 2.5) +
            deriv * (secondNumeratorFirstTerm / pow(denominator, 2.5) +
                secondNumeratorSecondTerm / pow(denominator, 3.5))
    }

    override fun toString() = "($ax*t^5+$bx*t^4+$cx*t^3+$dx*t^2+$ex*t+$fx,$ay*t^5+$by*t^4+$cy*t^3+$dy*t^2+$ey*t+$fy)"
}