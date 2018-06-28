package com.acmerobotics.library

import com.acmerobotics.library.path.parametric.QuinticPolynomial2d
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class SplineSegmentTest {
    @Test
    fun testSimpleSpline() {
        val spline = QuinticPolynomial2d(
            Waypoint2d(0.0, 0.0, 20.0, 0.0),
            Waypoint2d(12.0, 24.0, -20.0, 20.0)
        )
        GraphUtil.saveParametricCurve("simpleSpline2", spline)
    }
}