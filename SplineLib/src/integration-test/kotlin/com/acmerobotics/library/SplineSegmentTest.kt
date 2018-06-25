package com.acmerobotics.library

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class SplineSegmentTest {
    @Test
    fun testSimpleSpline() {
        val constraints = SimpleMotionConstraints(0.0, 0.0)
        val spline = SplineSegment(
            Waypoint(0.0, 0.0, 20.0, 0.0),
            Waypoint(12.0, 24.0, -20.0, 20.0),
            constraints
        )
        Graph.savePath("simpleSpline", spline)
    }
}