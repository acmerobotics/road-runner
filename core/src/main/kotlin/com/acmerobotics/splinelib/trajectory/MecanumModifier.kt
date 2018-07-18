package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Pose2d
import kotlin.math.abs

class MecanumModifier(trackWidth: Double, wheelbase: Double = trackWidth) : DriveModifier(4) {
    private val k = (trackWidth + wheelbase) / 2.0

    // TODO: verify this is actually correct
    override fun inverseKinematics(pose: Pose2d): List<Double> {
        println(pose)
        return listOf(
                pose.x - pose.y - k * pose.heading,
                pose.x + pose.y - k * pose.heading,
                pose.x - pose.y + k * pose.heading,
                pose.x + pose.y + k * pose.heading
        )
    }

    override fun getMaximumRobotVelocity(robotPoseDeriv: Pose2d, maxWheelVel: Double): Double {
        return maxWheelVel / (abs(robotPoseDeriv.x) + abs(robotPoseDeriv.y) + abs(k * robotPoseDeriv.heading))
    }
}