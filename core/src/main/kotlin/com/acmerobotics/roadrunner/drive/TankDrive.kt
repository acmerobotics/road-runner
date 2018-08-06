package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d

/**
 * This class provides basic functionality of a tank/differential drive using on [TankKinematics].
 *
 * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
 */
abstract class TankDrive(val trackWidth: Double) : Drive() {
    private var lastMotorPositions = emptyList<Double>()

    override fun setVelocity(poseVelocity: Pose2d) {
        val powers = TankKinematics.robotToWheelVelocities(poseVelocity, trackWidth)
        setMotorPowers(powers[0], powers[1])
    }

    override fun updatePoseEstimate() {
        val motorPositions = getMotorPositions()
        if (lastMotorPositions.isNotEmpty()) {
            val positionDeltas = motorPositions.zip(lastMotorPositions).map { it.first - it.second }
            val robotPoseDelta = TankKinematics.wheelToRobotVelocities(positionDeltas, trackWidth)
            val newHeading = poseEstimate.heading + robotPoseDelta.heading
            poseEstimate += Pose2d(robotPoseDelta.pos().rotated(newHeading), robotPoseDelta.heading)
        }
        lastMotorPositions = motorPositions
    }

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval [0.0, 1.0].
     */
    abstract fun setMotorPowers(left: Double, right: Double)

    /**
     * Returns the positions of the motors in radians.
     */
    abstract fun getMotorPositions(): List<Double>
}