package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.util.Angle

/**
 * This class provides the basic functionality of a swerve drive using [SwerveKinematics].
 *
 * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
 * @param wheelBase distance between pairs of wheels on the same side of the robot
 */
abstract class SwerveDrive @JvmOverloads constructor(
        val trackWidth: Double,
        val wheelBase: Double = trackWidth
) : Drive() {

    /**
     * Default localizer for swerve drives based on the drive encoder positions, module orientations, and (optionally) a
     * heading sensor.
     *
     * @param drive drive
     * @param useExternalHeading use external heading provided by an external sensor (e.g., IMU, gyroscope)
     */
    class SwerveLocalizer @JvmOverloads constructor(
            private val drive: SwerveDrive,
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
            val moduleOrientations = drive.getModuleOrientations()
            val extHeading = if (useExternalHeading) drive.getExternalHeading() else Double.NaN
            if (lastWheelPositions.isNotEmpty()) {
                val wheelDeltas = wheelPositions
                        .zip(lastWheelPositions)
                        .map { it.first - it.second }
                val robotPoseDelta = SwerveKinematics.wheelToRobotVelocities(
                        wheelDeltas, moduleOrientations, drive.wheelBase, drive.trackWidth)
                val finalHeadingDelta = if (useExternalHeading) Angle.norm(extHeading - lastExtHeading) else robotPoseDelta.heading
                poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, Pose2d(robotPoseDelta.pos(), finalHeadingDelta))
            }
            lastWheelPositions = wheelPositions
            lastExtHeading = extHeading
        }
    }

    override var localizer: Localizer = SwerveLocalizer(this)

    override fun setVelocity(poseVelocity: Pose2d) {
        val motorPowers = SwerveKinematics.robotToWheelVelocities(poseVelocity, trackWidth, wheelBase)
        val moduleOrientations = SwerveKinematics.robotToModuleOrientations(poseVelocity, trackWidth, wheelBase)
        setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3])
        setModuleOrientations(moduleOrientations[0], moduleOrientations[1], moduleOrientations[2], moduleOrientations[3])
    }

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval `[-1.0, 1.0]`.
     */
    abstract fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double)

    /**
     * Sets the module orientations. All values are in radians.
     */
    abstract fun setModuleOrientations(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double)

    /**
     * Returns the positions of the wheels in linear distance units. Positions should exactly match the ordering in
     * [setMotorPowers].
     */
    abstract fun getWheelPositions(): List<Double>

    /**
     * Returns the current module orientations in radians. Orientations should exactly match the order in
     * [setModuleOrientations].
     */
    abstract fun getModuleOrientations(): List<Double>

    /**
     * Returns the robot's heading in radians as measured by an external sensor (e.g., IMU, gyroscope). Heading is
     * measured counter-clockwise from the x-axis.
     */
    abstract fun getExternalHeading(): Double
}
