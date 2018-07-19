package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Pose2d
import kotlin.math.abs
import kotlin.math.min

class TankConstraints(baseConstraints: DriveConstraints, val trackWidth: Double) : DriveConstraints(baseConstraints.maximumVelocity, baseConstraints.maximumAcceleration, baseConstraints.maximumAngularVelocity, baseConstraints.maximumAngularAcceleration, baseConstraints.maximumCentripetalAcceleration) {
    override fun maximumVelocity(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double {
        val robotPositionDeriv = poseDeriv.pos().rotated(-pose.heading)

        val maxWheelVel = maximumVelocity / (robotPositionDeriv.x + abs(poseDeriv.heading * trackWidth / 2))

        return min(super.maximumVelocity(pose, poseDeriv, poseSecondDeriv), maxWheelVel)
    }

}