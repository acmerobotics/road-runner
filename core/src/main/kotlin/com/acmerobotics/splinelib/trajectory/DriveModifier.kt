package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Pose2d

abstract class DriveModifier(val numWheelProfiles: Int) {
    abstract fun inverseKinematics(pose: Pose2d): List<Double>

    abstract fun getMaximumRobotVelocity(robotPoseDeriv: Pose2d, maxWheelVel: Double): Double
}