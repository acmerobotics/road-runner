package com.acmerobotics.library

import com.acmerobotics.library.path.Path
import com.acmerobotics.library.path.QuinticSplineSegment
import com.acmerobotics.library.trajectory.DriveConstraints
import com.acmerobotics.library.trajectory.PathTrajectorySegment
import com.acmerobotics.library.trajectory.Trajectory
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class SplineTrajectoryTest {
    companion object {
        private val CONSTRAINTS = DriveConstraints(50.0, 25.0, Math.PI / 2, Math.PI / 2)
    }

    @Test
    fun testSimpleSpline() {
        val spline = QuinticSplineSegment(
            Waypoint(0.0, 0.0, 20.0, 20.0),
            Waypoint(30.0, 15.0, -30.0, 10.0)
        )
        val trajectory = Trajectory(listOf(
            PathTrajectorySegment(listOf(Path(spline)), listOf(CONSTRAINTS))
        ))

        GraphUtil.saveParametricCurve("simpleSpline/curve", spline)
        GraphUtil.saveTrajectory("simpleSpline/trajectory", trajectory)
    }

    @Test
    fun testCompositeSpline() {
        val spline = QuinticSplineSegment(
            Waypoint(15.0, 15.0, 15.0, 15.0),
            Waypoint(30.0, 15.0, 20.0, 5.0)
        )
        val trajectory = Trajectory(listOf(
            PathTrajectorySegment(listOf(Path(spline)), listOf(CONSTRAINTS))
        ))

        GraphUtil.saveParametricCurve("compositeSpline/curve", spline)
        GraphUtil.saveTrajectory("compositeSpline/trajectory", trajectory)
    }
}