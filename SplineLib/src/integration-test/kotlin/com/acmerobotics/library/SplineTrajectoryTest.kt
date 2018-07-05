package com.acmerobotics.library

import com.acmerobotics.library.spline.QuinticSpline
import com.acmerobotics.library.trajectory.DriveConstraints
import com.acmerobotics.library.trajectory.SplineTrajectorySegment
import com.acmerobotics.library.trajectory.Trajectory
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class SplineTrajectoryTest {
    companion object {
        private val CONSTRAINTS = DriveConstraints(50.0, 25.0, Math.PI / 2, Math.PI / 2)
    }

    @Test
    fun testStraightSpline() {
        val spline = QuinticSpline.fromPoses(
            Pose2d(0.0, 0.0, Math.PI / 4),
            Pose2d(15.0, 15.0, Math.PI / 4)
        )
        val trajectory = Trajectory(listOf(
            SplineTrajectorySegment(listOf(spline), listOf(CONSTRAINTS))
        ))

        GraphUtil.saveSpline("straightSpline/spline", spline)
        GraphUtil.saveTrajectory("straightSpline/trajectory", trajectory)
    }

    @Test
    fun testSimpleSpline() {
        val spline = QuinticSpline.fromPoses(
            Pose2d(0.0, 0.0, Math.PI / 4),
            Pose2d(30.0, 15.0, -3 * Math.PI / 4)
        )
        val trajectory = Trajectory(listOf(
            SplineTrajectorySegment(listOf(spline), listOf(CONSTRAINTS))
        ))

        GraphUtil.saveSpline("simpleSpline/spline", spline)
        GraphUtil.saveTrajectory("simpleSpline/trajectory", trajectory)
    }

    @Test
    fun testCompositeSpline() {
        val spline = QuinticSpline.fromPoses(
//            Pose2d(0.0, 0.0, Math.PI / 4),
            Pose2d(15.0, 15.0, Math.PI / 4),
            Pose2d(30.0, 15.0, 3 * Math.PI),
            Pose2d(20.0, 30.0, 0.0)
        )
        for (segment in spline.segments) {
            println(segment)
        }
        val trajectory = Trajectory(listOf(
            SplineTrajectorySegment(listOf(spline), listOf(CONSTRAINTS))
        ))

        GraphUtil.saveSpline("compositeSpline/spline", spline)
        GraphUtil.saveTrajectory("compositeSpline/trajectory", trajectory)
    }
}