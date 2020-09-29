package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.geometry.Pose2d

class TranslationalVelocityConstraint(
    private val maxTranslationalVel: Double
) : TrajectoryVelocityConstraint {
    override fun get(s: Double, pose: Pose2d, deriv: Pose2d, secondDeriv: Pose2d, baseVel: Pose2d) = maxTranslationalVel
}