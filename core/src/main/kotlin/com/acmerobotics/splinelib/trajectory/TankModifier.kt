package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Pose2d
import kotlin.math.abs

class TankModifier(val wheelbase: Double) : DriveModifier(2) {
    override fun inverseKinematics(pose: Pose2d): List<Double> {
        return listOf(
                pose.x + pose.heading * wheelbase / 2.0, // right
                pose.x - pose.heading * wheelbase / 2.0 // left
        )
    }

    // TODO: check Ryan's napkin calculations
    // robotPoseDeriv *in robot coordinates*
    override fun getMaximumRobotVelocity(robotPoseDeriv: Pose2d, maxWheelVel: Double) =
            maxWheelVel / (robotPoseDeriv.x + abs(robotPoseDeriv.heading * wheelbase / 2))
}