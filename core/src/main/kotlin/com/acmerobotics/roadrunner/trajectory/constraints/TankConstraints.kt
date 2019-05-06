package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.TankKinematics
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
    baseConstraints.maxVel,
    baseConstraints.maxAccel,
    baseConstraints.maxJerk,
    baseConstraints.maxAngVel,
    baseConstraints.maxAngAccel,
    baseConstraints.maxAngJerk
) {
    override operator fun get(pose: Pose2d, deriv: Pose2d, secondDeriv: Pose2d): SimpleMotionConstraints {
        val robotDeriv = Kinematics.fieldToRobotVelocity(pose, deriv)

        val wheelVelocities = TankKinematics.robotToWheelVelocities(robotDeriv, trackWidth)
        val maxTrajVel = wheelVelocities.map { maxVel / it }.map(::abs).min() ?: 0.0

        val superConstraints = super.get(pose, deriv, secondDeriv)

        return SimpleMotionConstraints(min(superConstraints.maxVel, maxTrajVel), superConstraints.maxAccel)
    }
}
