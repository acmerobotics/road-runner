@file:Suppress("NoItParamInMultilineLambda")

package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.kinematics.SwerveKinematics
import com.acmerobotics.roadrunner.kinematics.TankKinematics
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator
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

private val BASE_CONSTRAINTS = DriveConstraints(10.0, 25.0, Double.NaN, PI / 2, PI / 2, Double.NaN)

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DriveWheelConstraintsTest {
    @Test
    fun testTankWheelVelocityLimiting() {
        val constraints = TankConstraints(BASE_CONSTRAINTS, 10.0)
        val trajectory = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), constraints)
                .splineTo(Pose2d(15.0, 15.0, PI))
                .splineTo(Pose2d(5.0, 35.0, PI / 3))
                .build()

        val dt = trajectory.duration() / 10000.0
        val t = (0..10000).map { it * dt }
        val maxWheelVelMag = t.map {
            val pose = trajectory[it]
            val poseVel = trajectory.velocity(it)
            val robotVel = Kinematics.fieldToRobotVelocity(pose, poseVel)
            TankKinematics.robotToWheelVelocities(robotVel, 10.0)
                    .map(::abs)
                    .max() ?: 0.0
        }.max() ?: 0.0
        assertThat(maxWheelVelMag).isLessThan(BASE_CONSTRAINTS.maxVel + 0.1)
    }

    @Test
    fun testMecanumWheelVelocityLimiting() {
        val constraints = MecanumConstraints(BASE_CONSTRAINTS, 10.0, 5.0)
        val trajectory = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), constraints)
                .splineTo(Pose2d(15.0, 15.0, PI), interpolator = SplineInterpolator(0.0, PI / 2))
                .splineTo(Pose2d(5.0, 35.0, PI / 3), interpolator = TangentInterpolator())
                .build()

        val dt = trajectory.duration() / 10000.0
        val t = (0..10000).map { it * dt }
        val maxWheelVelMag = t.map {
            val pose = trajectory[it]
            val poseVel = trajectory.velocity(it)
            val robotVel = Kinematics.fieldToRobotVelocity(pose, poseVel)
            MecanumKinematics.robotToWheelVelocities(robotVel, 10.0, 5.0)
                    .map(::abs)
                    .max() ?: 0.0
        }.max() ?: 0.0
        assertThat(maxWheelVelMag).isLessThan(BASE_CONSTRAINTS.maxVel + 0.1)
    }

    @Test
    fun testSwerveWheelVelocityLimiting() {
        val constraints = SwerveConstraints(BASE_CONSTRAINTS, 10.0, 5.0)
        val trajectory = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), constraints)
                .splineTo(Pose2d(15.0, 15.0, PI), interpolator = SplineInterpolator(0.0, PI / 2))
                .splineTo(Pose2d(5.0, 35.0, PI / 3), interpolator = TangentInterpolator())
                .build()

        val dt = trajectory.duration() / 10000.0
        val t = (0..10000).map { it * dt }
        val maxWheelVelMag = t.map {
            val pose = trajectory[it]
            val poseVel = trajectory.velocity(it)
            val robotVel = Kinematics.fieldToRobotVelocity(pose, poseVel)
            SwerveKinematics.robotToWheelVelocities(robotVel, 10.0, 5.0)
                    .map(::abs)
                    .max() ?: 0.0
        }.max() ?: 0.0
        assertThat(maxWheelVelMag).isLessThan(BASE_CONSTRAINTS.maxVel + 0.1)
    }
}
