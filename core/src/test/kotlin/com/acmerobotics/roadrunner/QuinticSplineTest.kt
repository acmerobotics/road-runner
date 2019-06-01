package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.TestUtil.assertDerivEquals
import com.acmerobotics.roadrunner.path.QuinticSpline
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class QuinticSplineTest {

    @Suppress("LongMethod")
    @Test
    fun testSplineDerivatives() {
        val splineSegment = QuinticSpline(
                QuinticSpline.Waypoint(0.0, 0.0, 20.0, 40.0),
                QuinticSpline.Waypoint(45.0, 35.0, 60.0, 10.0)
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

        val t = s.map { splineSegment.reparam(it) }
        val dt = s.map { splineSegment.paramDeriv(it) }
        val d2t = s.map { splineSegment.paramSecondDeriv(it) }
        val d3t = s.map { splineSegment.paramThirdDeriv(it) }

        assertDerivEquals(x, dx, ds, 0.01)
        assertDerivEquals(dx, d2x, ds, 0.01)
        assertDerivEquals(d2x, d3x, ds, 0.01)

        assertDerivEquals(y, dy, ds, 0.01)
        assertDerivEquals(dy, d2y, ds, 0.01)
        assertDerivEquals(d2y, d3y, ds, 0.02)

        assertDerivEquals(tangentAngle, tangentAngleDeriv, ds, 0.0001)
        assertDerivEquals(tangentAngleDeriv, tangentAngleSecondDeriv, ds, 0.0001)

        assertDerivEquals(t, dt, ds, 0.03)
        assertDerivEquals(dt, d2t, ds, 0.02)
        assertDerivEquals(d2t, d3t, ds, 0.01)
    }

    @Test
    fun testInterpolation() {
        val splineSegment = QuinticSpline(
                QuinticSpline.Waypoint(0.0, 0.0, 20.0, 40.0),
                QuinticSpline.Waypoint(45.0, 35.0, 60.0, 10.0)
        )

        assertEquals(0.0, splineSegment[0.0].x, 1e-3)
        assertEquals(0.0, splineSegment[0.0].y, 1e-3)
        assertEquals(45.0, splineSegment[splineSegment.length()].x, 1e-3)
        assertEquals(35.0, splineSegment[splineSegment.length()].y, 1e-3)
    }

    @Test
    fun testDerivativeMagnitudeInvariance() {
        val splineSegment = QuinticSpline(
                QuinticSpline.Waypoint(0.0, 0.0, 20.0, 40.0),
                QuinticSpline.Waypoint(45.0, 35.0, 60.0, 10.0)
        )

        val splineSegment2 = QuinticSpline(
                QuinticSpline.Waypoint(0.0, 0.0, 40.0, 80.0),
                QuinticSpline.Waypoint(45.0, 35.0, 120.0, 20.0)
        )

        assertEquals(splineSegment.deriv(0.0).x, splineSegment2.deriv(0.0).x, 0.001)
        assertEquals(splineSegment.deriv(0.0).y, splineSegment2.deriv(0.0).y, 0.001)
        assertEquals(splineSegment.deriv(splineSegment.length()).x,
            splineSegment2.deriv(splineSegment.length()).x, 0.001)
        assertEquals(splineSegment.deriv(splineSegment.length()).y,
            splineSegment2.deriv(splineSegment.length()).y, 0.001)

        assertEquals(splineSegment.secondDeriv(0.0).x, splineSegment2.secondDeriv(0.0).x, 0.001)
        assertEquals(splineSegment.secondDeriv(0.0).y, splineSegment2.secondDeriv(0.0).y, 0.001)
        assertEquals(splineSegment.secondDeriv(splineSegment.length()).x,
            splineSegment2.secondDeriv(splineSegment.length()).x, 0.001)
        assertEquals(splineSegment.secondDeriv(splineSegment.length()).y,
            splineSegment2.secondDeriv(splineSegment.length()).y, 0.001)
    }
}
