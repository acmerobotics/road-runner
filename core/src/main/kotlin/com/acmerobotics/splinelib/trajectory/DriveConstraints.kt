package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Pose2d
import com.acmerobotics.splinelib.util.MathUtil
import kotlin.math.sqrt

/**
 * This class describes general robot trajectory constraints. More specifically, for paths, the robot wheel velocities,
 * robot acceleration, and centripetal acceleration are limited.  For point turns, the angular velocity and angular
 * acceleration are limited.
 */
open class DriveConstraints(
        val maximumVelocity: Double,
        val maximumAcceleration: Double,
        val maximumAngularVelocity: Double,
        val maximumAngularAcceleration: Double,
        val maximumCentripetalAcceleration: Double
) : TrajectoryConstraints {
    override fun maximumVelocity(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double {
        val maximumVelocities = mutableListOf(maximumVelocity)

        val curvature = MathUtil.curvature(poseDeriv.pos(), poseSecondDeriv.pos())
        maximumVelocities.add(sqrt(maximumCentripetalAcceleration / curvature))

        return maximumVelocities.min() ?: 0.0
    }

    override fun maximumAcceleration(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d) = maximumAcceleration
}
