package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.kinematics.SwerveKinematics
import com.acmerobotics.roadrunner.kinematics.TankKinematics
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.*
import com.acmerobotics.roadrunner.util.DoubleProgression
import org.assertj.core.api.Assertions.assertThat
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.PI
import kotlin.math.abs

private val ACCEL_CONSTRAINT = ProfileAccelerationConstraint(25.0)
private const val MAX_WHEEL_VEL = 10.0
private const val TRACK_WIDTH = 5.0

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DriveWheelConstraintsTest {
    private fun testWheelVelocityLimiting(
        velConstraint: TrajectoryVelocityConstraint,
        robotToWheelVelocities: (vel: Pose2d) -> List<Double>
    ) {
        val trajectory = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0),
            baseVelConstraint = velConstraint, baseAccelConstraint = ACCEL_CONSTRAINT)
            .splineTo(Vector2d(15.0, 15.0), PI)
            .splineTo(Vector2d(5.0, 35.0), PI / 3)
            .build()
        val t = DoubleProgression.fromClosedInterval(0.0, trajectory.duration(), 10_000)
        val maxWheelVelMag = t.map { time ->
                val pose = trajectory[time]
                val poseVel = trajectory.velocity(time)
                val robotVel = Kinematics.fieldToRobotVelocity(pose, poseVel)
                robotToWheelVelocities(robotVel)
                    .map(::abs)
                    .maxOrNull() ?: 0.0
            }.maxOrNull() ?: 0.0
        assertThat(maxWheelVelMag).isLessThan(MAX_WHEEL_VEL + 0.1)
    }

    @Test
    fun testTankWheelVelocityLimiting() {
        val velConstraint = TankVelocityConstraint(MAX_WHEEL_VEL, TRACK_WIDTH)

        testWheelVelocityLimiting(velConstraint) {
            TankKinematics.robotToWheelVelocities(it, TRACK_WIDTH)
        }
    }

    @Test
    fun testTankWheelVelocityLimitingReversed() {
        val velConstraint = TankVelocityConstraint(MAX_WHEEL_VEL, TRACK_WIDTH)

        testWheelVelocityLimiting(velConstraint) {
            TankKinematics.robotToWheelVelocities(it, TRACK_WIDTH)
        }
    }

    @Test
    fun testMecanumWheelVelocityLimiting() {
        val velConstraint = MecanumVelocityConstraint(MAX_WHEEL_VEL, TRACK_WIDTH)

        testWheelVelocityLimiting(velConstraint) {
            MecanumKinematics.robotToWheelVelocities(it, TRACK_WIDTH)
        }
    }

    @Test
    fun testSwerveWheelVelocityLimiting() {
        val velConstraint = SwerveVelocityConstraint(MAX_WHEEL_VEL, TRACK_WIDTH)

        testWheelVelocityLimiting(velConstraint) {
            SwerveKinematics.robotToWheelVelocities(it, TRACK_WIDTH)
        }
    }
}
