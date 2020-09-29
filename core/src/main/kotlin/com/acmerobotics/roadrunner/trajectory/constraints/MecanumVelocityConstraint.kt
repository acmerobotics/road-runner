package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import kotlin.math.abs

/**
 * Mecanum-specific drive constraints that also limit maximum wheel velocities.
 *
 * @param maxWheelVel maximum wheel velocity
 * @param trackWidth track width
 * @param wheelBase wheel base
 * @param lateralMultiplier lateral multiplier
 */
open class MecanumVelocityConstraint @JvmOverloads constructor(
    private val maxWheelVel: Double,
    private val trackWidth: Double,
    private val wheelBase: Double = trackWidth,
    private val lateralMultiplier: Double = 1.0
) : TrajectoryVelocityConstraint {
    override fun get(s: Double, pose: Pose2d, deriv: Pose2d, baseVel: Pose2d): Double {
        val robotDeriv = Kinematics.fieldToRobotVelocity(pose, deriv)

        val wheelVelocities = MecanumKinematics.robotToWheelVelocities(
            robotDeriv, trackWidth, wheelBase, lateralMultiplier)
        return wheelVelocities.map { maxWheelVel / it }.map(::abs).minOrNull()!!
    }
}
