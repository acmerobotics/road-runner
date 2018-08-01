package com.acmerobotics.splinelib.drive

import com.acmerobotics.splinelib.Pose2d

// TODO: implement the full set of Drive methods once the API is more stable
abstract class TankDrive(val trackWidth: Double) : Drive {
    override fun setVelocity(poseVelocity: Pose2d) {
        val powers = TankKinematics.robotToWheelVelocities(poseVelocity, trackWidth)
        setMotorPowers(powers[0], powers[1])
    }

    abstract fun setMotorPowers(left: Double, right: Double)
}