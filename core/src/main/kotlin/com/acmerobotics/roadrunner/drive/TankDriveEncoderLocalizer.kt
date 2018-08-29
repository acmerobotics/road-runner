package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.util.NanoClock

class TankDriveEncoderLocalizer @JvmOverloads constructor(
        private val drive: TankDrive,
        private val clock: NanoClock = NanoClock.system()
) : Localizer {
    override var poseEstimate: Pose2d = Pose2d()
        set(value) {
            lastWheelPositions = emptyList()
            lastUpdateTimestamp = Double.NaN
            field = value
        }
    private var lastWheelPositions = emptyList<Double>()
    private var lastUpdateTimestamp = Double.NaN

    override fun update() {
        val wheelPositions = drive.getWheelPositions()
        val timestamp = clock.seconds()
        if (lastWheelPositions.isNotEmpty()) {
            val dt = timestamp - lastUpdateTimestamp
            val wheelVelocities = wheelPositions
                    .zip(lastWheelPositions)
                    .map { (it.first - it.second) / dt }
            val robotPoseDelta = TankKinematics.wheelToRobotVelocities(wheelVelocities, drive.trackWidth) * dt
            poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, robotPoseDelta)
        }
        lastWheelPositions = wheelPositions
        lastUpdateTimestamp = timestamp
    }
}