package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d

/**
 * This class provides basic functionality of a mecanum drive using on [MecanumKinematics].
 *
 * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
 * @param wheelBase distance between pairs of wheels on the same side of the robot
 */
abstract class MecanumDrive @JvmOverloads constructor(
        val trackWidth: Double,
        val wheelBase: Double = trackWidth
) : Drive() {
    private var lastMotorPositions = emptyList<Double>()

    override fun setVelocity(poseVelocity: Pose2d) {
        val powers = MecanumKinematics.robotToWheelVelocities(poseVelocity, trackWidth, wheelBase)
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    override fun updatePoseEstimate() {
        val motorPositions = getMotorPositions()
        if (lastMotorPositions.isNotEmpty()) {
            val positionDeltas = motorPositions.zip(lastMotorPositions).map { it.first - it.second }
            val robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(positionDeltas, wheelBase, trackWidth)
            val newHeading = poseEstimate.heading + robotPoseDelta.heading
            poseEstimate += Pose2d(robotPoseDelta.pos().rotated(newHeading), robotPoseDelta.heading)
        }
        lastMotorPositions = motorPositions
    }

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval [0.0, 1.0].
     */
    abstract fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double)

    /**
     * Returns the positions of the motors in radians.
     */
    abstract fun getMotorPositions(): List<Double>
}