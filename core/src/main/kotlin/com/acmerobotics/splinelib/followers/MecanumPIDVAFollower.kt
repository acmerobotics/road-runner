package com.acmerobotics.splinelib.followers

import com.acmerobotics.splinelib.Pose2d
import com.acmerobotics.splinelib.control.PIDCoefficients
import com.acmerobotics.splinelib.control.PIDFController
import com.acmerobotics.splinelib.drive.MecanumDrive
import com.acmerobotics.splinelib.drive.MecanumKinematics
import kotlin.math.sign

/**
 * Traditional PID controller with feedforward velocity and acceleration components to follow a trajectory. More
 * specifically, the feedback is applied to the components of the robot's pose (x position, y position, and heading) to
 * determine the velocity correction. The feedforward components are instead applied at the wheel level.
 */
class MecanumPIDVAFollower(
        private val drive: MecanumDrive,
        translationalCoeffs: PIDCoefficients,
        headingCoeffs: PIDCoefficients,
        private val kV: Double,
        private val kA: Double,
        private val kStatic: Double
) : TrajectoryFollower() {
    private val axialController = PIDFController(translationalCoeffs)
    private val lateralController = PIDFController(translationalCoeffs)
    private val headingController = PIDFController(headingCoeffs)

    override fun internalUpdate(currentPose: Pose2d, currentTimestamp: Double) {
        val t = elapsedTime(currentTimestamp)

        val targetPose = trajectory[t]
        val targetPoseVelocity = trajectory.velocity(t)
        val targetPoseAcceleration = trajectory.acceleration(t)

        val targetRobotPose = Pose2d(targetPose.pos().rotated(-targetPose.heading), targetPose.heading)
        val targetRobotPoseVelocity = Pose2d(targetPoseVelocity.pos().rotated(-targetPose.heading), targetPoseVelocity.heading)
        val targetRobotPoseAcceleration = Pose2d(targetPoseAcceleration.pos().rotated(-targetPose.heading), targetPoseAcceleration.heading)
        val currentRobotPose = Pose2d(currentPose.pos().rotated(-currentPose.heading), currentPose.heading)

        axialController.targetPosition = targetRobotPose.x
        lateralController.targetPosition = targetRobotPose.y
        headingController.targetPosition = targetRobotPose.heading

        val axialCorrection = axialController.update(currentRobotPose.x, targetRobotPoseVelocity.x)
        val lateralCorrection = lateralController.update(currentRobotPose.y, targetRobotPoseVelocity.y)
        val headingCorrection = headingController.update(currentRobotPose.heading, targetRobotPoseVelocity.heading)

        val correctedVelocity = targetRobotPoseVelocity + Pose2d(axialCorrection, lateralCorrection, headingCorrection)

        val wheelVelocities = MecanumKinematics.robotToWheelVelocities(correctedVelocity, drive.trackWidth, drive.wheelBase)
        val wheelAccelerations = MecanumKinematics.robotToWheelAccelerations(targetRobotPoseAcceleration, drive.trackWidth, drive.wheelBase)

        val motorPowers = wheelVelocities
                .zip(wheelAccelerations)
                .map { it.first * kV + it.second * kA }
                .map { it + sign(it) * kStatic }
        drive.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3])
    }
}