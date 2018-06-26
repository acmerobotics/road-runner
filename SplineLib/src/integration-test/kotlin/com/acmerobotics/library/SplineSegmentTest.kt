package com.acmerobotics.library

import com.acmerobotics.library.path.SplineSegment
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class SplineSegmentTest {
    @Test
    fun testSimpleSpline() {
        val spline = SplineSegment(
            Waypoint(0.0, 0.0, 20.0, 0.0),
            Waypoint(12.0, 24.0, -20.0, 20.0)
        )
        GraphUtil.savePath("simpleSpline2", spline)
    }
}