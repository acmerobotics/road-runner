package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.geometry.Pose2d
import kotlin.math.abs

class AngularVelocityConstraint(
    private val maxAngularVel: Double
) : TrajectoryVelocityConstraint {
    override fun get(s: Double, pose: Pose2d, deriv: Pose2d, baseVel: Pose2d) =
        abs(maxAngularVel / deriv.heading)
}