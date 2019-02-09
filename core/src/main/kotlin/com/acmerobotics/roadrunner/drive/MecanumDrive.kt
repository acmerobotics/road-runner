package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle

/**
 * This class provides the basic functionality of a mecanum drive using [MecanumKinematics].
 *
 * @param kV velocity feedforward
 * @param kA acceleration feedforward
 * @param kStatic additive constant feedforward
 * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
 * @param wheelBase distance between pairs of wheels on the same side of the robot
 */
abstract class MecanumDrive @JvmOverloads constructor(
    private val kV: Double,
    private val kA: Double,
    private val kStatic: Double,
    private val trackWidth: Double,
    private val wheelBase: Double = trackWidth
) : Drive() {

    /**
     * Default localizer for mecanum drives based on the drive encoders and (optionally) a heading sensor.
     *
     * @param drive drive
     * @param useExternalHeading use external heading provided by an external sensor (e.g., IMU, gyroscope)
     */
    class MecanumLocalizer @JvmOverloads constructor(
        private val drive: MecanumDrive,
        private val useExternalHeading: Boolean = true
    ) : Localizer {
        override var poseEstimate: Pose2d = Pose2d()
            set(value) {
                lastWheelPositions = emptyList()
                lastExtHeading = Double.NaN
                drive.externalHeading = value.heading
                field = value
            }
        private var lastWheelPositions = emptyList<Double>()
        private var lastExtHeading = Double.NaN

        override fun update() {
            val wheelPositions = drive.getWheelPositions()
            val extHeading = if (useExternalHeading) drive.externalHeading else Double.NaN
            if (lastWheelPositions.isNotEmpty()) {
                val wheelDeltas = wheelPositions
                    .zip(lastWheelPositions)
                    .map { it.first - it.second }
                val robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(
                    wheelDeltas, drive.wheelBase, drive.trackWidth
                )
                val finalHeadingDelta = if (useExternalHeading)
                    Angle.norm(extHeading - lastExtHeading)
                else
                    robotPoseDelta.heading
                poseEstimate = Kinematics.relativeOdometryUpdate(
                    poseEstimate,
                    Pose2d(robotPoseDelta.pos(), finalHeadingDelta)
                )
            }
            lastWheelPositions = wheelPositions
            lastExtHeading = extHeading
        }
    }

    override var localizer: Localizer = MecanumLocalizer(this)

    override fun setDriveSignal(driveSignal: DriveSignal) {
        val velocities = MecanumKinematics.robotToWheelVelocities(driveSignal.velocity, trackWidth, wheelBase)
        val accelerations = MecanumKinematics.robotToWheelAccelerations(driveSignal.acceleration, trackWidth, wheelBase)
        val powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic)
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
}
