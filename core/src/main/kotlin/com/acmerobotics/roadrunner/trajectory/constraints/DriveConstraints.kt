package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.SimpleMotionConstraints
import kotlin.math.abs

/**
 * This class describes general robot trajectory constraints. More specifically, for paths, the robot velocity,
 * robot acceleration, and robot angular velocity are limited.  For point turns, the angular velocity and angular
 * acceleration are limited.
 *
 * @param maxVel maximum robot velocity
 * @param maxAccel maximum robot acceleration
 * @param maxJerk maximum robot jerk (ignored by dynamic profiles)
 * @param maxAngVel maximum angular velocity
 * @param maxAngAccel maximum angular acceleration (ignored by path-based trajectories)
 * @param maxAngJerk maximum angular jerk (ignored by path-based trajectories)
 */
open class DriveConstraints(
    @JvmField var maxVel: Double,
    @JvmField var maxAccel: Double,
    @JvmField var maxJerk: Double,
    @JvmField var maxAngVel: Double,
    @JvmField var maxAngAccel: Double,
    @JvmField var maxAngJerk: Double
) : TrajectoryConstraints {
    override fun get(pose: Pose2d, deriv: Pose2d, secondDeriv: Pose2d): SimpleMotionConstraints {
        val maxVels = mutableListOf(maxVel)

        if (abs(deriv.heading) > 1e-6) {
            maxVels.add(maxAngVel / Math.abs(deriv.heading))
        }

        return SimpleMotionConstraints(maxVels.min() ?: 0.0, maxAccel)
    }
}
