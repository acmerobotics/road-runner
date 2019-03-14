package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.LineSegment
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.path.QuinticSpline
import com.acmerobotics.roadrunner.trajectory.PathTrajectorySegment
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints
import kotlin.math.PI
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

private val BASE_CONSTRAINTS = DriveConstraints(50.0, 25.0, PI / 2, PI / 2)
private val CONSTRAINTS = TankConstraints(BASE_CONSTRAINTS, 12.0)

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class SplineTrajectoryTest {
    @Test
    fun testLineSegment() {
        val line = LineSegment(
            Vector2d(0.0, 0.0),
            Vector2d(25.0, 25.0)
        )
        val trajectory = Trajectory(listOf(
                PathTrajectorySegment(listOf(Path(line)), listOf(BASE_CONSTRAINTS))
        ))

        GraphUtil.saveParametricCurve("lineSegment/curve", line)
        GraphUtil.saveTrajectory("lineSegment/trajectory", trajectory)
    }

    @Test
    fun testSimpleSpline() {
        val spline = QuinticSpline(
                QuinticSpline.Waypoint(0.0, 0.0, 20.0, 20.0),
                QuinticSpline.Waypoint(30.0, 15.0, -30.0, 10.0)
        )
        val trajectory = Trajectory(listOf(
            PathTrajectorySegment(listOf(Path(spline)), listOf(BASE_CONSTRAINTS))
        ))

        GraphUtil.saveParametricCurve("simpleSpline/curve", spline)
        GraphUtil.saveTrajectory("simpleSpline/trajectory", trajectory)
    }

    @Test
    fun testCompositeSpline() {
        val line = LineSegment(
            Vector2d(0.0, 0.0),
            Vector2d(15.0, 15.0)
        )
        val spline = QuinticSpline(
                QuinticSpline.Waypoint(15.0, 15.0, 15.0, 15.0),
                QuinticSpline.Waypoint(30.0, 15.0, 20.0, 5.0)
        )
        val trajectory = Trajectory(listOf(
            PathTrajectorySegment(listOf(Path(line), Path(spline)), listOf(CONSTRAINTS, CONSTRAINTS))
        ))

        GraphUtil.saveParametricCurve("compositeSpline/curve", spline)
        GraphUtil.saveTrajectory("compositeSpline/trajectory", trajectory)
    }
}
