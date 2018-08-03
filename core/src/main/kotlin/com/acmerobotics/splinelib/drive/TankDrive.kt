package com.acmerobotics.splinelib.drive

import com.acmerobotics.splinelib.Pose2d

abstract class TankDrive(val trackWidth: Double) : Drive {
    private var internalPoseEstimate = Pose2d()
    private var lastPoseUpdateTimestamp = Double.NaN
    private var lastMotorPositions = emptyList<Double>()

    override fun setVelocity(poseVelocity: Pose2d) {
        val powers = TankKinematics.robotToWheelVelocities(poseVelocity, trackWidth)
        setMotorPowers(powers[0], powers[1])
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
            val robotPoseDelta = TankKinematics.wheelToRobotVelocities(positionDeltas, trackWidth)
            val newHeading = internalPoseEstimate.heading + robotPoseDelta.heading
            internalPoseEstimate += Pose2d(robotPoseDelta.pos().rotated(newHeading), robotPoseDelta.heading)
        }
        lastMotorPositions = motorPositions
        lastPoseUpdateTimestamp = timestamp
    }

    abstract fun setMotorPowers(left: Double, right: Double)
    abstract fun getMotorPositions(): List<Double>
}