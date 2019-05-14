package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.TestUtil.assertDerivEquals
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator
import com.acmerobotics.roadrunner.path.heading.WiggleInterpolator
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.PI

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class TrajectoryTest {
    @Test
    fun testTrajectoryDerivatives() {
        val cryptoColWidth = 7.5
        val stonePose = Pose2d(48.0, -47.5, PI)
        val trajectory = TrajectoryBuilder(stonePose, DriveConstraints(5.0, 10.0, Double.NaN, 2.0, 3.0, Double.NaN))
                .lineTo(Vector2d(12 - cryptoColWidth, -44.0))
                .splineTo(Pose2d(16.0, -24.0, PI / 3))
                .splineTo(
                    Pose2d(24.0, -10.0, PI / 4),
                        WiggleInterpolator(Math.toRadians(15.0), 6.0, TangentInterpolator()))
                .build()

        val dt = trajectory.duration() / 10000.0
        val t = (0..10000).map { it * dt }

        val x = t.map { trajectory[it].x }
        val velX = t.map { trajectory.velocity(it).x }
        val accelX = t.map { trajectory.acceleration(it).x }

        val y = t.map { trajectory[it].y }
        val velY = t.map { trajectory.velocity(it).y }
        val accelY = t.map { trajectory.acceleration(it).y }

        assertDerivEquals(x, velX, dt, 0.01)
        assertDerivEquals(velX, accelX, dt, 0.01)

        assertDerivEquals(y, velY, dt, 0.01)
        assertDerivEquals(velY, accelY, dt, 0.01)
    }
}
