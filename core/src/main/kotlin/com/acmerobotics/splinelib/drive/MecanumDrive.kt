package com.acmerobotics.splinelib.drive

import com.acmerobotics.splinelib.Pose2d

abstract class MecanumDrive(val trackWidth: Double, val wheelBase: Double = trackWidth) : Drive {
    private var internalPoseEstimate = Pose2d()
    private var lastPoseUpdateTimestamp = Double.NaN
    private var lastMotorPositions = emptyList<Double>()

    override fun setVelocity(poseVelocity: Pose2d) {
        val powers = MecanumKinematics.robotToWheelVelocities(poseVelocity, trackWidth, wheelBase)
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    override fun getPoseEstimate() = internalPoseEstimate

    override fun resetPoseEstimate(newPose: Pose2d) {
        internalPoseEstimate = newPose
        lastPoseUpdateTimestamp = Double.NaN
    }

    override fun updatePoseEstimate(timestamp: Double) {
        val motorPositions = getMotorPositions()
        if (!lastPoseUpdateTimestamp.isNaN()) {
            val positionDeltas = motorPositions.zip(lastMotorPositions).map { it.first - it.second }
            val robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(positionDeltas, wheelBase, trackWidth)
            val newHeading = internalPoseEstimate.heading + robotPoseDelta.heading
            internalPoseEstimate += Pose2d(robotPoseDelta.pos().rotated(newHeading), robotPoseDelta.heading)
        }
        lastMotorPositions = motorPositions
        lastPoseUpdateTimestamp = timestamp
    }

    abstract fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double)
    abstract fun getMotorPositions(): List<Double>
}