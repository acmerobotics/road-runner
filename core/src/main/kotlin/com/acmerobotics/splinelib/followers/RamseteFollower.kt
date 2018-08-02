package com.acmerobotics.splinelib.followers

import com.acmerobotics.splinelib.Pose2d
import com.acmerobotics.splinelib.drive.TankDrive
import com.acmerobotics.splinelib.drive.TankKinematics
import kotlin.math.*

/**
 * Time-varying, non-linear feedback controller for nonholonomic drives. See equation 5.12 of
 * [Ramsete01.pdf](https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf).
 *
 * @param drive tank drive
 * @param b b parameter (non-negative)
 * @param zeta zeta parameter (on (0, 1))
 * @param kV feedforward velocity gain
 * @param kA feedforward acceleration gain (currently unused)
 * @param kStatic additive feedforward constant (used to overcome static friction)
 */
class RamseteFollower(
        private val drive: TankDrive,
        private val b: Double,
        private val zeta: Double,
        private val kV: Double,
        private val kA: Double,
        private val kStatic: Double
) : TrajectoryFollower() {
    override fun internalUpdate(currentPose: Pose2d, currentTimestamp: Double) {
        val t = elapsedTime(currentTimestamp)

        val targetPose = trajectory[t]
        val targetPoseVelocity = trajectory.velocity(t)

        val targetRobotPose = Pose2d(targetPose.pos().rotated(-targetPose.heading), targetPose.heading)
        val currentRobotPose = Pose2d(currentPose.pos().rotated(-currentPose.heading), currentPose.heading)
        val targetRobotPoseVelocity = Pose2d(targetPoseVelocity.pos().rotated(-targetPose.heading), targetPoseVelocity.heading)

        val targetV = targetRobotPoseVelocity.x
        val targetOmega = targetRobotPoseVelocity.heading
        val error = targetRobotPose - currentRobotPose

        val k1 = 2 * zeta * sqrt(targetOmega * targetOmega + b * targetV * targetV)
        val k3 = k1
        val k2 = b

        val v = targetRobotPoseVelocity.x * cos(error.heading) +
                k1 * (cos(currentRobotPose.heading) * error.x + sin(currentRobotPose.heading) * error.y)
        val omega = targetOmega + k2 * targetV * sin(error.heading) / error.heading *
                (cos(currentPose.heading) * error.x - sin(currentPose.heading) * error.y) +
                k3 * error.heading

        // TODO: is Ramsete acceleration FF worth?
        val targetRobotPoseAcceleration = Pose2d()

        val wheelVelocities = TankKinematics.robotToWheelVelocities(Pose2d(v, 0.0, omega), drive.trackWidth)
        val wheelAccelerations = TankKinematics.robotToWheelAccelerations(targetRobotPoseAcceleration, drive.trackWidth)

        val motorPowers = wheelVelocities
                .zip(wheelAccelerations)
                .map { it.first * kV + it.second * kA }
                .map { it + sign(it) * kStatic }
        drive.setMotorPowers(motorPowers[0], motorPowers[1])    }
}