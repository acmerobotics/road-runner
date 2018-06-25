package com.acmerobotics.library

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class TrajectoryTest {
    @Test
    fun testSimpleSpline() {
        val constraints = DriveMotionConstraints(10.0, 5.0, maximumAngularVelocity = 0.5, maximumCentripetalAcceleration = 2.0)
        val spline = SplineSegment(
            Waypoint(0.0, 0.0, 20.0, 0.0),
            Waypoint(12.0, 24.0, -4.0, 20.0),
            constraints
        )
        val path = HolonomicPath(spline, TangentInterpolator())
        val trajectory = Trajectory(listOf(PathTrajectorySegment(listOf(path), listOf(constraints))))
        GraphUtil.saveTrajectory("simpleSpline", trajectory)
        GraphUtil.saveMotionProfile("simpleSpline", (trajectory.segments[0] as PathTrajectorySegment).profile, false)
        CSVUtil.saveHolonomicPath("simpleSpline", path)
    }
}