package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.geometry.Pose2d
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
 * @param lateralMultiplier lateral multiplier
 */
open class MecanumConstraints @JvmOverloads constructor(
    baseConstraints: DriveConstraints,
    val trackWidth: Double,
    val wheelBase: Double = trackWidth,
    val lateralMultiplier: Double = 1.0
) : DriveConstraints(
    baseConstraints.maxVel,
    baseConstraints.maxAccel,
    baseConstraints.maxJerk,
    baseConstraints.maxAngVel,
    baseConstraints.maxAngAccel,
    baseConstraints.maxAngJerk
) {
    override operator fun get(s: Double, pose: Pose2d, deriv: Pose2d, secondDeriv: Pose2d): SimpleMotionConstraints {
        val robotDeriv = Kinematics.fieldToRobotVelocity(pose, deriv)

        val wheelVelocities = MecanumKinematics.robotToWheelVelocities(robotDeriv, trackWidth, wheelBase, lateralMultiplier)
        val maxTrajVel = wheelVelocities.map { maxVel / it }.map(::abs).min() ?: 0.0

        val superConstraints = super.get(s, pose, deriv, secondDeriv)

        return SimpleMotionConstraints(min(superConstraints.maxVel, maxTrajVel), superConstraints.maxAccel)
    }
}
