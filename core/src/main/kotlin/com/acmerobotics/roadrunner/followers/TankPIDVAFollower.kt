package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.Kinematics
import com.acmerobotics.roadrunner.drive.TankDrive
import com.acmerobotics.roadrunner.drive.TankKinematics
import com.acmerobotics.roadrunner.util.NanoClock

/**
 * Traditional PID controller with feedforward velocity and acceleration components to follow a trajectory. More
 * specifically, one feedback loop controls the path displacement (that is, x in the robot reference frame), and
 * another feedback loop to minimize cross track (lateral) error via heading correction (overall, very similar to
 * [MecanumPIDVAFollower] except adjusted for the nonholonomic constraint). Feedforward is applied at the wheel level.
 *
 * @param drive tank drive instance
 * @param displacementCoeffs PID coefficients for the robot axial (x) controller
 * @param crossTrackCoeffs PID coefficients for the robot heading controller based on cross track error
 * @param kV feedforward velocity gain
 * @param kA feedforward acceleration gain
 * @param kStatic signed, additive feedforward constant (used to overcome static friction)
 * @param admissibleError admissible/satisfactory pose error at the end of each move
 * @param timeout max time to wait for the error to be admissible
 * @param clock clock
 */
class TankPIDVAFollower @JvmOverloads constructor(
        private val drive: TankDrive,
        displacementCoeffs: PIDCoefficients,
        crossTrackCoeffs: PIDCoefficients,
        private val kV: Double,
        private val kA: Double,
        private val kStatic: Double,
        admissibleError: Pose2d = Pose2d(),
        timeout: Double = 0.0,
        clock: NanoClock = NanoClock.system()
) : TrajectoryFollower(admissibleError, timeout, clock) {
    private val displacementController = PIDFController(displacementCoeffs)
    private val crossTrackController = PIDFController(crossTrackCoeffs)

    override var lastError: Pose2d = Pose2d()

    override fun update(currentPose: Pose2d) {
        super.update(currentPose)

        if (!isFollowing()) {
            drive.setMotorPowers(0.0, 0.0)
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
        displacementController.targetPosition = poseError.x
        crossTrackController.targetPosition = poseError.y

        // note: feedforward is processed at the wheel level; velocity is only passed here to adjust the derivative term
        val axialCorrection = displacementController.update(0.0, targetRobotPoseVelocity.x)
        val headingCorrection = crossTrackController.update(0.0, targetRobotPoseVelocity.y)

        val correctedVelocity = targetRobotPoseVelocity + Pose2d(axialCorrection, 0.0, headingCorrection)

        val wheelVelocities = TankKinematics.robotToWheelVelocities(correctedVelocity, drive.trackWidth)
        val wheelAccelerations = TankKinematics.robotToWheelAccelerations(targetRobotPoseAcceleration, drive.trackWidth)

        val motorPowers = Kinematics.calculateMotorFeedforward(wheelVelocities, wheelAccelerations, kV, kA, kStatic)

        drive.setMotorPowers(motorPowers[0], motorPowers[1])

        lastError = poseError
    }
}