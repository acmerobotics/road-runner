package com.acmerobotics.library

import com.acmerobotics.library.path.Path
import com.acmerobotics.library.path.heading.TangentInterpolator
import com.acmerobotics.library.path.parametric.QuinticPolynomial2d
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
        val spline = QuinticPolynomial2d.fromPoses(
            Pose2d(0.0, 0.0, Math.PI / 6),
            Pose2d(108.0, 72.0, 3 * Math.PI / 4)
        )
        println(hypot(40.0, 200.0))
        println(hypot(108.0, 72.0))
        val path = Path(spline, TangentInterpolator())
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
        GraphUtil.saveParametricCurve("simpleSpline", spline)
        CSVUtil.savePath("simpleSpline", path)
    }
}