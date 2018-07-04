package com.acmerobotics.library

import com.acmerobotics.library.path.parametric.QuinticSpline
import com.acmerobotics.library.trajectory.DriveConstraints
import com.acmerobotics.library.trajectory.SplineTrajectorySegment
import com.acmerobotics.library.trajectory.Trajectory
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class TrajectoryTest {
    @Test
    fun testSimpleSpline() {
        val constraints = DriveConstraints(
            50.0,
            25.0,
            maximumAngularVelocity = 2 * Math.PI,
            maximumAngularAcceleration = Math.PI
        )
        val spline = QuinticSpline.fromPoses(
            Pose2d(0.0, 0.0, Math.PI / 6),
//            Pose2d(-3.0, 7.2, 3 * Math.PI / 4),
            Pose2d(2.4, 3.2, -Math.PI)
        )
        val trajectory = Trajectory(
            listOf(
                SplineTrajectorySegment(
                    listOf(spline),
                    listOf(constraints)
                )
            )
        )
        GraphUtil.saveTrajectory("simpleSpline", trajectory, 100000)
        GraphUtil.saveSplineDerivatives("simpleSpline", spline, 100000)
        GraphUtil.saveSpline("simpleSpline", spline)
        CSVUtil.saveSpline("simpleSpline", spline)
    }
}