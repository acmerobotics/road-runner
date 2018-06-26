package com.acmerobotics.library

import com.acmerobotics.library.path.HolonomicPath
import com.acmerobotics.library.path.SplineSegment
import com.acmerobotics.library.path.TangentInterpolator
import com.acmerobotics.library.trajectory.DriveMotionConstraints
import com.acmerobotics.library.trajectory.PathTrajectorySegment
import com.acmerobotics.library.trajectory.Trajectory
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class TrajectoryTest {
    @Test
    fun testSimpleSpline() {
        val constraints = DriveMotionConstraints(
            50.0,
            500.0,
            maximumCentripetalAcceleration = 15.0
        )
        val spline = SplineSegment(
            Waypoint(0.0, 0.0, 200.0, 0.0),
            Waypoint(108.0, 72.0, -40.0, 200.0),
            constraints
        )
        val path = HolonomicPath(spline, TangentInterpolator())
        val trajectory = Trajectory(
            listOf(
                PathTrajectorySegment(
                    listOf(path),
                    listOf(constraints),
                    10000
                )
            )
        )
        GraphUtil.saveTrajectory("simpleSpline", trajectory)
        GraphUtil.saveMotionProfile("simpleSpline", (trajectory.segments[0] as PathTrajectorySegment).profile, false)
        GraphUtil.savePath("simpleSpline", spline)
        CSVUtil.saveHolonomicPath("simpleSpline", path)
    }
}