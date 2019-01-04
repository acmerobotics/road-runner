package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.Drive
import com.acmerobotics.roadrunner.drive.Kinematics
import com.acmerobotics.roadrunner.util.NanoClock

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
 * @param admissibleError admissible/satisfactory pose error at the end of each move
 * @param timeout max time to wait for the error to be admissible
 * @param clock clock
 */
abstract class HolonomicPIDVAFollower @JvmOverloads constructor(
        private val drive: Drive,
        translationalCoeffs: PIDCoefficients,
        headingCoeffs: PIDCoefficients,
        private val kV: Double,
        private val kA: Double,
        private val kStatic: Double,
        admissibleError: Pose2d = Pose2d(),
        timeout: Double = 0.0,
        clock: NanoClock = NanoClock.system()
) : TrajectoryFollower(admissibleError, timeout, clock) {
    private val axialController = PIDFController(translationalCoeffs)
    private val lateralController = PIDFController(translationalCoeffs)
    private val headingController = PIDFController(headingCoeffs)

    override var lastError: Pose2d = Pose2d()

    init {
        headingController.setInputBounds(-Math.PI, Math.PI)
    }

    override fun update(currentPose: Pose2d) {
        super.update(currentPose)

        if (!isFollowing()) {
            drive.setVelocity(Pose2d(0.0, 0.0, 0.0))
            return
        }

        val t = elapsedTime()

        val targetPose = trajectory[t]
        val targetPoseVelocity = trajectory.velocity(t)
        val targetPoseAcceleration = trajectory.acceleration(t)

        val targetRobotPoseVelocity = Kinematics.fieldToRobotPoseVelocity(targetPose, targetPoseVelocity)
        val targetRobotPoseAcceleration = Kinematics.fieldToRobotPoseAcceleration(targetPose, targetPoseVelocity, targetPoseAcceleration)

        val poseError = Kinematics.calculatePoseError(targetPose, currentPose)

        // you can pass the error directly to PIDFController by setting setpoint = error and position = 0
        axialController.targetPosition = poseError.x
        lateralController.targetPosition = poseError.y
        headingController.targetPosition = poseError.heading

        // note: feedforward is processed at the wheel level; velocity is only passed here to adjust the derivative term
        val axialCorrection = axialController.update(0.0, targetRobotPoseVelocity.x)
        val lateralCorrection = lateralController.update(0.0, targetRobotPoseVelocity.y)
        val headingCorrection = headingController.update(0.0, targetRobotPoseVelocity.heading)

        val correctedVelocity = targetRobotPoseVelocity + Pose2d(axialCorrection, lateralCorrection, headingCorrection)

        updateDrive(correctedVelocity, targetRobotPoseAcceleration)

        lastError = poseError
    }

    /**
     * Update the drive powers.
     *
     * @param poseVelocity robot pose velocity
     * @param poseAcceleration robot pose acceleration
     */
    abstract fun updateDrive(poseVelocity: Pose2d, poseAcceleration: Pose2d)
}