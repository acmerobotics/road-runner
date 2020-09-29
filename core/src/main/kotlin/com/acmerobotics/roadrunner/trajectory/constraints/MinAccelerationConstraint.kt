package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.geometry.Pose2d

class MinAccelerationConstraint(
    private val constraints: List<TrajectoryAccelerationConstraint>
) : TrajectoryAccelerationConstraint {
    override fun get(s: Double, pose: Pose2d, deriv: Pose2d, baseVel: Pose2d) =
        constraints.map { it[s, pose, deriv, baseVel] }.minOrNull()!!
}