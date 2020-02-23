package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.kinematics.SwerveKinematics
import com.acmerobotics.roadrunner.kinematics.TankKinematics
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.SwerveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints
import org.assertj.core.api.Assertions.assertThat
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.PI
import kotlin.math.abs

private val BASE_CONSTRAINTS = DriveConstraints(10.0, 25.0, 0.0, PI / 2, PI / 2, 0.0)

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DriveWheelConstraintsTest {
    private fun testWheelVelocityLimiting(
        trajectory: Trajectory,
        maxWheelVel: Double,
        robotToWheelVelocities: (vel: Pose2d) -> List<Double>
    ) {
        val dt = trajectory.duration() / 10000.0
        val t = (0..10000).map { it * dt }
        val maxWheelVelMag = t.map { time ->
            val pose = trajectory[time]
            val poseVel = trajectory.velocity(time)
            val robotVel = Kinematics.fieldToRobotVelocity(pose, poseVel)
            robotToWheelVelocities(robotVel)
                .map(::abs)
                .max() ?: 0.0
        }.max() ?: 0.0
        assertThat(maxWheelVelMag).isLessThan(maxWheelVel + 0.1)
    }

    @Test
    fun testTankWheelVelocityLimiting() {
        val constraints = TankConstraints(BASE_CONSTRAINTS, 10.0)
        val trajectory = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), constraints = constraints)
            .splineTo(Pose2d(15.0, 15.0, PI))
            .splineTo(Pose2d(5.0, 35.0, PI / 3))
            .build()
        testWheelVelocityLimiting(trajectory, BASE_CONSTRAINTS.maxVel) {
            TankKinematics.robotToWheelVelocities(it, constraints.trackWidth)
        }
    }

    @Test
    fun testTankWheelVelocityLimitingReversed() {
        val constraints = TankConstraints(BASE_CONSTRAINTS, 10.0)
        val trajectory = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), startHeading = PI, constraints = constraints)
            .splineTo(Pose2d(15.0, 15.0, PI))
            .splineTo(Pose2d(5.0, 35.0, PI / 3))
            .build()
        testWheelVelocityLimiting(trajectory, BASE_CONSTRAINTS.maxVel) {
            TankKinematics.robotToWheelVelocities(it, constraints.trackWidth)
        }
    }

    @Test
    fun testMecanumWheelVelocityLimiting() {
        val constraints = MecanumConstraints(BASE_CONSTRAINTS, 10.0, 5.0)
        val trajectory = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), constraints = constraints)
            .splineToSplineHeading(Pose2d(15.0, 15.0, PI), PI / 2)
            .splineTo(Pose2d(5.0, 35.0, PI / 3))
            .build()
        testWheelVelocityLimiting(trajectory, BASE_CONSTRAINTS.maxVel) {
            MecanumKinematics.robotToWheelVelocities(it, constraints.trackWidth, constraints.wheelBase)
        }
    }

    @Test
    fun testSwerveWheelVelocityLimiting() {
        val constraints = SwerveConstraints(BASE_CONSTRAINTS, 10.0, 5.0)
        val trajectory = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), constraints = constraints)
            .splineToSplineHeading(Pose2d(15.0, 15.0, PI), PI / 2)
            .splineTo(Pose2d(5.0, 35.0, PI / 3))
            .build()
        testWheelVelocityLimiting(trajectory, BASE_CONSTRAINTS.maxVel) {
            SwerveKinematics.robotToWheelVelocities(it, constraints.trackWidth, constraints.wheelBase)
        }
    }
}
