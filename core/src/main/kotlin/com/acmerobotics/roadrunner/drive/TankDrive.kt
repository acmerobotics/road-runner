package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.NanoClock

/**
 * This class provides the basic functionality of a tank/differential drive using [TankKinematics].
 *
 * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
 */
abstract class TankDrive @JvmOverloads constructor(
        val trackWidth: Double,
        clock: NanoClock = NanoClock.system()
) : Drive() {

    /**
     * Default localizer for tank drives based on the drive encoders and (optionally) a heading sensor.
     *
     * @param drive drive
     * @param useExternalHeading use external heading provided by an external sensor (e.g., IMU, gyroscope)
     * @param clock clock
     */
    class TankLocalizer @JvmOverloads constructor(
            private val drive: TankDrive,
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
                val robotPoseDelta = TankKinematics.wheelToRobotVelocities(wheelVelocities, drive.trackWidth) * dt
                val finalHeadingDelta = if (useExternalHeading) Angle.norm(extHeading - lastExtHeading) else robotPoseDelta.heading
                poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, Pose2d(robotPoseDelta.pos(), finalHeadingDelta))
            }
            lastWheelPositions = wheelPositions
            lastExtHeading = extHeading
            lastUpdateTimestamp = timestamp
        }
    }

    override var localizer: Localizer = TankLocalizer(this, clock = clock)

    override fun setVelocity(poseVelocity: Pose2d) {
        val powers = TankKinematics.robotToWheelVelocities(poseVelocity, trackWidth)
        setMotorPowers(powers[0], powers[1])
    }

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval `[-1.0, 1.0]`.
     */
    abstract fun setMotorPowers(left: Double, right: Double)

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