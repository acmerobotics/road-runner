package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.path.PathSegment
import com.acmerobotics.roadrunner.path.QuinticSpline
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.util.DoubleProgression
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.PI

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class TrajectoryTest {
    @Test
    fun testTrajectoryDerivatives() {
        val cryptoColWidth = 7.5
        val stonePose = Pose2d(48.0, -47.5, PI)
        val trajectory = TrajectoryBuilder(stonePose, constraints = DriveConstraints(5.0, 10.0, 0.0, 2.0, 3.0, 0.0))
                .lineTo(Vector2d(12 - cryptoColWidth, -47.5))
                .splineTo(Vector2d(16.0, -24.0), PI / 3)
                .build()

        val t = DoubleProgression.fromClosedInterval(0.0, trajectory.duration(), 10_000)

        val x = t.map { trajectory[it].x }
        val velX = t.map { trajectory.velocity(it).x }
        val accelX = t.map { trajectory.acceleration(it).x }

        val y = t.map { trajectory[it].y }
        val velY = t.map { trajectory.velocity(it).y }
        val accelY = t.map { trajectory.acceleration(it).y }

        // there is a lot of noise in these numerical derivatives from the new parametrization
        // however the analytic ones are perfect
        TestUtil.assertDerivEquals(x, velX, t.step, 0.05, 0.1)
        TestUtil.assertDerivEquals(velX, accelX, t.step, 0.05, 0.1)

        TestUtil.assertDerivEquals(y, velY, t.step, 0.05, 0.1)
        TestUtil.assertDerivEquals(velY, accelY, t.step, 0.05, 0.1)
    }

    @Test
    fun testShortTrajectory() {
        val path = Path(listOf(PathSegment(QuinticSpline(
                QuinticSpline.Knot(0.0, 0.0, 0.0, -1.0),
                QuinticSpline.Knot(1e-4, 1e-4, 0.707, 0.707)
        )), PathSegment(QuinticSpline(
                QuinticSpline.Knot(1e-4, 1e-4, 0.707, 0.707),
                QuinticSpline.Knot(2e-4, 0.0, -1.0, 0.0)
        ))))
        TrajectoryGenerator.generateTrajectory(path, DriveConstraints(5.0, 10.0, 0.0, 2.0, 3.0, 0.0), resolution = 1.0)
    }

    @Test
    fun testTrajectoryEnd() {
        val endPose = Pose2d(25.0, 25.0, 0.0)
        repeat(50) {
            val traj = TrajectoryBuilder(Pose2d(), constraints = DriveConstraints(5.0, 10.0, 0.0, 2.0, 3.0, 0.0))
                .splineTo(Vector2d(50 * Math.random(), 50 * Math.random()), 2 * PI * Math.random())
                .splineTo(Vector2d(50 * Math.random(), 50 * Math.random()), 2 * PI * Math.random())
                .splineTo(endPose.vec(), endPose.heading)
                .build()
            assert(traj.end() epsilonEqualsHeading endPose)
        }
    }
}
