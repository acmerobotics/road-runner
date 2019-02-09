package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.profile.SimpleMotionConstraints
import kotlin.math.abs
import kotlin.math.min

/**
 * Mecanum-specific drive constraints that also limit maximum wheel velocities.
 *
 * @param baseConstraints base drive constraints
 * @param trackWidth track width
 * @param wheelBase wheel base
 */
open class MecanumConstraints @JvmOverloads constructor(
    baseConstraints: DriveConstraints,
    private val trackWidth: Double,
    private val wheelBase: Double = trackWidth
) : DriveConstraints(
        baseConstraints.maximumVelocity,
        baseConstraints.maximumAcceleration,
        baseConstraints.maximumAngularVelocity,
        baseConstraints.maximumAngularAcceleration
) {
    override operator fun get(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): SimpleMotionConstraints {
        val robotPoseDeriv = Kinematics.fieldToRobotPoseVelocity(pose, poseDeriv)

        val wheelVelocities = MecanumKinematics.robotToWheelVelocities(robotPoseDeriv, trackWidth, wheelBase)
        val maxTrajVel = wheelVelocities.map { maximumVelocity / it }.map(::abs).min() ?: 0.0

        val superConstraints = super.get(pose, poseDeriv, poseSecondDeriv)

        return SimpleMotionConstraints(min(superConstraints.maximumVelocity, maxTrajVel), superConstraints.maximumAcceleration)
    }
}