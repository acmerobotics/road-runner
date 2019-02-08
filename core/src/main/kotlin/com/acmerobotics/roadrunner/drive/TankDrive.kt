package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.DriveSignal
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.util.Angle

/**
 * This class provides the basic functionality of a tank/differential drive using [TankKinematics].
 *
 * @param kV velocity feedforward
 * @param kA acceleration feedforward
 * @param kStatic additive constant feedforward
 * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
 */
abstract class TankDrive constructor(
    private val kV: Double,
    private val kA: Double,
    private val kStatic: Double,
    private val trackWidth: Double
) : Drive() {

    /**
     * Default localizer for tank drives based on the drive encoders and (optionally) a heading sensor.
     *
     * @param drive drive
     * @param useExternalHeading use external heading provided by an external sensor (e.g., IMU, gyroscope)
     */
    class TankLocalizer @JvmOverloads constructor(
            private val drive: TankDrive,
            private val useExternalHeading: Boolean = true
    ) : Localizer {
        override var poseEstimate: Pose2d = Pose2d()
            set(value) {
                lastWheelPositions = emptyList()
                lastExtHeading = Double.NaN
                field = value
            }
        private var lastWheelPositions = emptyList<Double>()
        private var lastExtHeading = Double.NaN

        override fun update() {
            val wheelPositions = drive.getWheelPositions()
            val extHeading = if (useExternalHeading) drive.getExternalHeading() else Double.NaN
            if (lastWheelPositions.isNotEmpty()) {
                val wheelDeltas = wheelPositions
                        .zip(lastWheelPositions)
                        .map { it.first - it.second }
                val robotPoseDelta = TankKinematics.wheelToRobotVelocities(wheelDeltas, drive.trackWidth)
                val finalHeadingDelta = if (useExternalHeading) Angle.norm(extHeading - lastExtHeading) else robotPoseDelta.heading
                poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, Pose2d(robotPoseDelta.pos(), finalHeadingDelta))
            }
            lastWheelPositions = wheelPositions
            lastExtHeading = extHeading
        }
    }

    override var localizer: Localizer = TankLocalizer(this)

    override fun setDriveSignal(driveSignal: DriveSignal) {
        val velocities = TankKinematics.robotToWheelVelocities(driveSignal.velocity, trackWidth)
        val accelerations = TankKinematics.robotToWheelAccelerations(driveSignal.acceleration, trackWidth)
        val powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic)
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