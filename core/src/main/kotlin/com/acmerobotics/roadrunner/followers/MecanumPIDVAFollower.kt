package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.drive.MecanumKinematics
import com.acmerobotics.roadrunner.util.NanoClock
import kotlin.math.sign

/**
 * Traditional PID controller with feedforward velocity and acceleration components to follow a trajectory. More
 * specifically, the feedback is applied to the components of the robot's pose (x position, y position, and heading) to
 * determine the velocity correction. The feedforward components are instead applied at the wheel level.
 *
 * @param drive mecanum drive instance
 * @param translationalCoeffs PID coefficients for the robot axial and lateral (x and y, respectively) controllers
 * @param headingCoeffs PID coefficients for the robot heading controller
 * @param kV feedforward velocity gain
 * @param kA feedforward acceleration gain
 * @param kStatic signed, additive feedforward constant (used to overcome static friction)
 */
class MecanumPIDVAFollower @JvmOverloads constructor(
        private val drive: MecanumDrive,
        translationalCoeffs: PIDCoefficients,
        headingCoeffs: PIDCoefficients,
        private val kV: Double,
        private val kA: Double,
        private val kStatic: Double,
        clock: NanoClock = NanoClock.default()
) : TrajectoryFollower(clock) {
    private val axialController = PIDFController(translationalCoeffs)
    private val lateralController = PIDFController(translationalCoeffs)
    private val headingController = PIDFController(headingCoeffs)

    override fun update(currentPose: Pose2d) {
        if (!isFollowing()) {
            drive.setMotorPowers(0.0, 0.0, 0.0, 0.0)
            return
        }

        val t = elapsedTime()

        val targetPose = trajectory[t]
        val targetPoseVelocity = trajectory.velocity(t)
        val targetPoseAcceleration = trajectory.acceleration(t)

        val targetRobotPose = Pose2d(targetPose.pos().rotated(-targetPose.heading), 0.0)
        val targetRobotPoseVelocity = Pose2d(targetPoseVelocity.pos().rotated(-targetPose.heading), targetPoseVelocity.heading)
        val targetRobotPoseAcceleration = Pose2d(targetPoseAcceleration.pos().rotated(-targetPose.heading), targetPoseAcceleration.heading) +
                Pose2d(-targetPoseVelocity.x * Math.sin(targetPose.heading) + targetPoseVelocity.y * Math.cos(targetPose.heading),
                        -targetPoseVelocity.x * Math.cos(targetPose.heading) - targetPoseVelocity.y * Math.sin(targetPose.heading),
                        0.0
                ) * targetPoseVelocity.heading
        val currentRobotPose = Pose2d(currentPose.pos().rotated(-targetPose.heading), currentPose.heading - targetPose.heading)

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