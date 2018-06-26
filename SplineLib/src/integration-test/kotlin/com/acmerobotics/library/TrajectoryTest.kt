package com.acmerobotics.library

import com.acmerobotics.library.path.HolonomicPath
import com.acmerobotics.library.path.SplineSegment
import com.acmerobotics.library.path.TangentInterpolator
import com.acmerobotics.library.trajectory.DriveMotionConstraints
import com.acmerobotics.library.trajectory.PathTrajectorySegment
import com.acmerobotics.library.trajectory.Trajectory
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import java.lang.Math.hypot


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class TrajectoryTest {
    @Test
    fun testSimpleSpline() {
        val constraints = DriveMotionConstraints(
            50.0,
            500.0,
            maximumCentripetalAcceleration = 15.0
        )
        val spline = SplineSegment.fromPoses(
            Pose2d(0.0, 0.0, Math.PI / 6),
            Pose2d(108.0, 72.0, 3 * Math.PI / 4)
        )
        println(hypot(40.0, 200.0))
        println(hypot(108.0, 72.0))
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