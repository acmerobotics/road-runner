package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.util.NanoClock

/**
 * This class provides basic functionality of a mecanum drive using on [MecanumKinematics].
 *
 * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
 * @param wheelBase distance between pairs of wheels on the same side of the robot
 */
abstract class MecanumDrive @JvmOverloads constructor(
        val trackWidth: Double,
        val wheelBase: Double = trackWidth,
        clock: NanoClock = NanoClock.system()
) : Drive() {
    override var localizer: Localizer = MecanumDriveEncoderLocalizer(this, clock)

    override fun setVelocity(poseVelocity: Pose2d) {
        val powers = MecanumKinematics.robotToWheelVelocities(poseVelocity, trackWidth, wheelBase)
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval `[-1.0, 1.0]`.
     */
    abstract fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double)

    /**
     * Returns the positions of the wheels in linear distance units.
     */
    abstract fun getWheelPositions(): List<Double>
}