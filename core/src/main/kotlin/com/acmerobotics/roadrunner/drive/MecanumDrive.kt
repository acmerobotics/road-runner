package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d

abstract class MecanumDrive @JvmOverloads constructor(val trackWidth: Double, val wheelBase: Double = trackWidth) : Drive {
    private var internalPoseEstimate = Pose2d()
    private var lastMotorPositions = emptyList<Double>()

    override fun setVelocity(poseVelocity: Pose2d) {
        val powers = MecanumKinematics.robotToWheelVelocities(poseVelocity, trackWidth, wheelBase)
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    override fun getPoseEstimate() = internalPoseEstimate

    override fun resetPoseEstimate(newPose: Pose2d) {
        internalPoseEstimate = newPose
    }

    override fun updatePoseEstimate() {
        val motorPositions = getMotorPositions()
        if (lastMotorPositions.isNotEmpty()) {
            val positionDeltas = motorPositions.zip(lastMotorPositions).map { it.first - it.second }
            val robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(positionDeltas, wheelBase, trackWidth)
            val newHeading = internalPoseEstimate.heading + robotPoseDelta.heading
            internalPoseEstimate += Pose2d(robotPoseDelta.pos().rotated(newHeading), robotPoseDelta.heading)
        }
        lastMotorPositions = motorPositions
    }

    abstract fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double)
    abstract fun getMotorPositions(): List<Double>
}