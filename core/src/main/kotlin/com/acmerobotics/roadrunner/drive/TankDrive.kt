package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.util.NanoClock

/**
 * This class provides basic functionality of a tank/differential drive using on [TankKinematics].
 *
 * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
 */
abstract class TankDrive(val trackWidth: Double, clock: NanoClock = NanoClock.system()) : Drive() {
    override var localizer: Localizer = TankDriveEncoderLocalizer(this, clock)

    override fun setVelocity(poseVelocity: Pose2d) {
        val powers = TankKinematics.robotToWheelVelocities(poseVelocity, trackWidth)
        setMotorPowers(powers[0], powers[1])
    }

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval `[-1.0, 1.0]`.
     */
    abstract fun setMotorPowers(left: Double, right: Double)

    /**
     * Returns the positions of the wheels in linear distance units.
     */
    abstract fun getWheelPositions(): List<Double>
}