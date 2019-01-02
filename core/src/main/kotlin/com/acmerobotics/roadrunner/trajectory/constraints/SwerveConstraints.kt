package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.drive.Kinematics
import com.acmerobotics.roadrunner.drive.SwerveKinematics
import kotlin.math.abs
import kotlin.math.min

/**
 * Mecanum-specific drive constraints that also limit maximum wheel velocities.
 *
 * @param baseConstraints base drive constraints
 * @param trackWidth track width
 * @param wheelBase wheel base
 */
open class SwerveConstraints @JvmOverloads constructor(
        baseConstraints: DriveConstraints,
        private val trackWidth: Double,
        private val wheelBase: Double = trackWidth
) : DriveConstraints(
        baseConstraints.maximumVelocity,
        baseConstraints.maximumAcceleration,
        baseConstraints.maximumAngularVelocity,
        baseConstraints.maximumAngularAcceleration
) {
    override fun maximumVelocity(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double {
        val robotPoseDeriv = Kinematics.fieldToRobotPoseVelocity(pose, poseDeriv)

        val wheelVelocities = SwerveKinematics.robotToWheelVelocities(robotPoseDeriv, trackWidth, wheelBase)
        val maxTrajVel = wheelVelocities.map { maximumVelocity / it }.map(::abs).min() ?: 0.0

        return min(super.maximumVelocity(pose, poseDeriv, poseSecondDeriv), maxTrajVel)
    }

}