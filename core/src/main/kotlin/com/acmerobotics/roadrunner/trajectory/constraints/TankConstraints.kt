package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.drive.Kinematics
import com.acmerobotics.roadrunner.drive.TankKinematics
import com.acmerobotics.roadrunner.profile.SimpleMotionConstraints
import kotlin.math.abs
import kotlin.math.min

/**
 * Tank-specific drive constraints that also limit maximum wheel velocities.
 *
 * @param baseConstraints base drive constraints
 * @param trackWidth track width
 */
open class TankConstraints(
        baseConstraints: DriveConstraints,
        val trackWidth: Double
) : DriveConstraints(
        baseConstraints.maximumVelocity,
        baseConstraints.maximumAcceleration,
        baseConstraints.maximumAngularVelocity,
        baseConstraints.maximumAngularAcceleration
) {
    override operator fun get(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): SimpleMotionConstraints {
        val robotPoseDeriv = Kinematics.fieldToRobotPoseVelocity(pose, poseDeriv)

        val wheelVelocities = TankKinematics.robotToWheelVelocities(robotPoseDeriv, trackWidth)
        val maxTrajVel = wheelVelocities.map { maximumVelocity / it }.map(::abs).min() ?: 0.0

        val superConstraints = super.get(pose, poseDeriv, poseSecondDeriv)

        return SimpleMotionConstraints(min(superConstraints.maximumVelocity, maxTrajVel), superConstraints.maximumAcceleration)
    }

}