package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Pose2d
import kotlin.math.abs

/**
 * This class describes general robot trajectory constraints. More specifically, for paths, the robot wheel velocities,
 * robot acceleration, and robot angular velocity are limited.  For point turns, the angular velocity and angular
 * acceleration are limited.
 */
open class DriveConstraints(
        val maximumVelocity: Double,
        val maximumAcceleration: Double,
        val maximumAngularVelocity: Double,
        val maximumAngularAcceleration: Double
) : TrajectoryConstraints {
    companion object {
        private const val EPSILON = 1e-6
    }

    override fun maximumVelocity(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double {
        val maximumVelocities = mutableListOf(maximumVelocity)

        if (abs(poseDeriv.heading) > EPSILON) {
            maximumVelocities.add(maximumAngularVelocity / Math.abs(poseDeriv.heading))
        }

        return maximumVelocities.min() ?: 0.0
    }

    override fun maximumAcceleration(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d) = maximumAcceleration
}
