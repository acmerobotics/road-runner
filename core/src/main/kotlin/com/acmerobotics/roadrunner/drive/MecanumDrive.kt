package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.NanoClock

/**
 * This class provides the basic functionality of a mecanum drive using [MecanumKinematics].
 *
 * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
 * @param wheelBase distance between pairs of wheels on the same side of the robot
 */
abstract class MecanumDrive @JvmOverloads constructor(
        val trackWidth: Double,
        val wheelBase: Double = trackWidth,
        clock: NanoClock = NanoClock.system()
) : Drive() {

    /**
     * Default localizer for mecanum drives based on the drive encoders and (optionally) a heading sensor.
     *
     * @param drive drive
     * @param useExternalHeading use external heading provided by an external sensor (e.g., IMU, gyroscope)
     * @param clock clock
     */
    class MecanumLocalizer @JvmOverloads constructor(
            private val drive: MecanumDrive,
            private val useExternalHeading: Boolean = true,
            private val clock: NanoClock = NanoClock.system()
    ) : Localizer {
        override var poseEstimate: Pose2d = Pose2d()
            set(value) {
                lastWheelPositions = emptyList()
                lastExtHeading = Double.NaN
                lastUpdateTimestamp = Double.NaN
                field = value
            }
        private var lastWheelPositions = emptyList<Double>()
        private var lastExtHeading = Double.NaN
        private var lastUpdateTimestamp = Double.NaN

        override fun update() {
            val wheelPositions = drive.getWheelPositions()
            val extHeading = if (useExternalHeading) drive.getExternalHeading() else Double.NaN
            val timestamp = clock.seconds()
            if (lastWheelPositions.isNotEmpty()) {
                val dt = timestamp - lastUpdateTimestamp
                val wheelVelocities = wheelPositions
                        .zip(lastWheelPositions)
                        .map { (it.first - it.second) / dt }
                val robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(wheelVelocities, drive.wheelBase, drive.trackWidth) * dt
                val finalHeadingDelta = if (useExternalHeading) Angle.norm(extHeading - lastExtHeading) else robotPoseDelta.heading
                poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, Pose2d(robotPoseDelta.pos(), finalHeadingDelta))
            }
            lastWheelPositions = wheelPositions
            lastExtHeading = extHeading
            lastUpdateTimestamp = timestamp
        }
    }

    override var localizer: Localizer = MecanumLocalizer(this, clock = clock)

    override fun setVelocity(poseVelocity: Pose2d) {
        val powers = MecanumKinematics.robotToWheelVelocities(poseVelocity, trackWidth, wheelBase)
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval `[-1.0, 1.0]`.
     */
    abstract fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double)

    /**
     * Returns the positions of the wheels in linear distance units. Positions should exactly match the ordering in
     * [setMotorPowers].
     */
    abstract fun getWheelPositions(): List<Double>

    /**
     * Returns the robot's heading in radians as measured by an external sensor (e.g., IMU, gyroscope). Heading is
     * measured counter-clockwise from the x-axis.
     */
    abstract fun getExternalHeading(): Double
}