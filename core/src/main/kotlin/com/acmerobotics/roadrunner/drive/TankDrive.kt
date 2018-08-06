package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d

/**
 * This class provides basic functionality of a tank/differential drive using on [TankKinematics].
 *
 * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
 */
abstract class TankDrive(val trackWidth: Double) : Drive() {
    private var lastWheelPositions = emptyList<Double>()

    override fun setVelocity(poseVelocity: Pose2d) {
        val powers = TankKinematics.robotToWheelVelocities(poseVelocity, trackWidth)
        setMotorPowers(powers[0], powers[1])
    }

    override fun updatePoseEstimate() {
        val wheelPositions = getWheelPositions()
        if (lastWheelPositions.isNotEmpty()) {
            val positionDeltas = wheelPositions
                    .zip(lastWheelPositions)
                    .map { it.first - it.second }
            val robotPoseDelta = TankKinematics.wheelToRobotVelocities(positionDeltas, trackWidth)
            val newHeading = poseEstimate.heading + robotPoseDelta.heading
            poseEstimate += Pose2d(robotPoseDelta.pos().rotated(newHeading), robotPoseDelta.heading)
        }
        lastWheelPositions = wheelPositions
    }

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval `[0.0, 1.0]`.
     */
    abstract fun setMotorPowers(left: Double, right: Double)

    /**
     * Returns the positions of the wheels in linear distance units.
     */
    abstract fun getWheelPositions(): List<Double>
}