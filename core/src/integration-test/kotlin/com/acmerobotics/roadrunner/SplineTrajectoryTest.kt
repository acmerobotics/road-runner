package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator
import com.acmerobotics.roadrunner.trajectory.constraints.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.PI

private val VEL_CONSTRAINT = MinVelocityConstraint(
    listOf(
        TankVelocityConstraint(50.0, 12.0),
        AngularVelocityConstraint(PI / 2)
    )
)
private val ACCEL_CONSTRAINT = ProfileAccelerationConstraint(25.0)

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class SplineTrajectoryTest {
    @Test
    fun testLineSegment() {
        val line = LineSegment(
            Vector2d(0.0, 0.0),
            Vector2d(25.0, 25.0)
        )
        val trajectory = TrajectoryGenerator.generateTrajectory(
            Path(PathSegment(line)),
            VEL_CONSTRAINT,
            ACCEL_CONSTRAINT
        )

        GraphUtil.saveParametricCurve("lineSegment/curve", line)
        GraphUtil.saveTrajectory("lineSegment/trajectory", trajectory)
    }

    @Test
    fun testSimpleSpline() {
        val spline = QuinticSpline(
            QuinticSpline.Knot(0.0, 0.0, 20.0, 20.0),
            QuinticSpline.Knot(30.0, 15.0, -30.0, 10.0)
        )
        val trajectory = TrajectoryGenerator.generateTrajectory(
            Path(PathSegment(spline)),
            VEL_CONSTRAINT,
            ACCEL_CONSTRAINT
        )

        GraphUtil.saveParametricCurve("Sample Quintic Spline", spline)
        GraphUtil.saveTrajectory("simpleSpline/trajectory", trajectory)
    }

    @Test
    fun testCompositeSpline() {
        val line = LineSegment(
            Vector2d(0.0, 0.0),
            Vector2d(15.0, 15.0)
        )
        val spline = QuinticSpline(
            QuinticSpline.Knot(15.0, 15.0, 15.0, 15.0),
            QuinticSpline.Knot(30.0, 15.0, 20.0, 5.0)
        )
        val path = Path(
            listOf(
                PathSegment(line),
                PathSegment(spline)
            )
        )
        val trajectory = TrajectoryGenerator.generateTrajectory(path, VEL_CONSTRAINT, ACCEL_CONSTRAINT)

        GraphUtil.saveParametricCurve("compositeSpline/curve", spline)
        GraphUtil.saveTrajectory("compositeSpline/trajectory", trajectory)
    }
}
