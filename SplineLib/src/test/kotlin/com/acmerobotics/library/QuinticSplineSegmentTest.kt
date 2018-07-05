package com.acmerobotics.library

import com.acmerobotics.library.spline.QuinticSplineSegment
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.abs

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class QuinticSplineSegmentTest {
    companion object {
        fun compareDerivatives(x: List<Double>, dx: List<Double>, ds: Double, epsilon: Double): Boolean {
            val numDx = (0 until x.size - 2).map { (x[it+2] - x[it]) / (2 * ds) }
            for (i in 2 until x.size - 2) {
                if (abs(numDx[i - 1] - dx[i]) > epsilon) {
                    return false
                }
            }
            return true
        }
    }

    @Test
    fun testSplineDerivatives() {
        val splineSegment = QuinticSplineSegment(
            Waypoint(0.0, 0.0, 20.0, 40.0),
            Waypoint(45.0, 35.0, 60.0, 10.0)
        )
        val resolution = 1000
        val ds = splineSegment.length() / resolution.toDouble()
        val s = (0..resolution).map { it * ds }

        val x = s.map { splineSegment[it].x }
        val dx = s.map { splineSegment.deriv(it).x }
        val d2x = s.map { splineSegment.secondDeriv(it).x }
        val d3x = s.map { splineSegment.thirdDeriv(it).x }

        val y = s.map { splineSegment[it].y }
        val dy = s.map { splineSegment.deriv(it).y }
        val d2y = s.map { splineSegment.secondDeriv(it).y }
        val d3y = s.map { splineSegment.thirdDeriv(it).y }

        val tangentAngle = s.map { splineSegment.tangentAngle(it) }
        val tangentAngleDeriv = s.map { splineSegment.tangentAngleDeriv(it) }
        val tangentAngleSecondDeriv = s.map { splineSegment.tangentAngleSecondDeriv(it) }

        val t = s.map { splineSegment.displacementToParameter(it) }
        val dt = s.map { splineSegment.parameterDeriv(it) }
        val d2t = s.map { splineSegment.parameterSecondDeriv(it) }
        val d3t = s.map { splineSegment.parameterThirdDeriv(it) }

        assert(compareDerivatives(x, dx, ds, 0.01))
        assert(compareDerivatives(dx, d2x, ds, 0.01))
        assert(compareDerivatives(d2x, d3x, ds, 0.01))

        assert(compareDerivatives(y, dy, ds, 0.01))
        assert(compareDerivatives(dy, d2y, ds, 0.01))
        assert(compareDerivatives(d2y, d3y, ds, 0.02))

        assert(compareDerivatives(tangentAngle, tangentAngleDeriv, ds, 0.01))
        assert(compareDerivatives(tangentAngleDeriv, tangentAngleSecondDeriv, ds, 0.01))

        assert(compareDerivatives(t, dt, ds, 0.03))
        assert(compareDerivatives(dt, d2t, ds, 0.05))
        assert(compareDerivatives(d2t, d3t, ds, 0.01))
    }
}