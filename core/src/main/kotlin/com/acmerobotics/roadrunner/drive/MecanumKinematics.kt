package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d
import org.apache.commons.math3.linear.MatrixUtils

/**
 * Mecanum drive kinematic equations. [Ether's paper](http://www.chiefdelphi.com/media/papers/download/2722) provides a
 * derivation.
 */
// TODO: does unrolling mecanum kinematic matrix multiplies obscure intent?
// the answer seems to be no for tank/differential
object MecanumKinematics {

    /**
     * Computes the wheel velocities corresponding to [robotPoseVelocity] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotPoseVelocity velocity of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     */
    // TODO: it might be better to just pass K to better facilitate experimental results
    // alternatively, they can just pass k as trackWidth
    @JvmStatic
    @JvmOverloads
    fun robotToWheelVelocities(robotPoseVelocity: Pose2d, trackWidth: Double, wheelBase: Double = trackWidth): List<Double> {
        val k = (trackWidth + wheelBase) / 2.0
        val inverseKinematicsMatrix = MatrixUtils.createRealMatrix(
                arrayOf(
                        doubleArrayOf(1.0, -1.0, -k),
                        doubleArrayOf(1.0, 1.0, -k),
                        doubleArrayOf(1.0, -1.0, k),
                        doubleArrayOf(1.0, 1.0, k)
                )
        )
        val poseMatrix = MatrixUtils.createColumnRealMatrix(doubleArrayOf(
                robotPoseVelocity.x, robotPoseVelocity.y, robotPoseVelocity.heading))
        val wheelVelocities = inverseKinematicsMatrix.multiply(poseMatrix)
        return wheelVelocities.transpose().data[0].toList()
    }

    /**
     * Computes the wheel accelerations corresponding to [robotPoseAcceleration] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotPoseAcceleration acceleration of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     */
    // follows from linearity of the derivative
    @JvmStatic
    @JvmOverloads
    fun robotToWheelAccelerations(robotPoseAcceleration: Pose2d, trackWidth: Double, wheelBase: Double = trackWidth) =
            robotToWheelVelocities(robotPoseAcceleration, trackWidth, wheelBase)

    /**
     * Computes the robot velocity corresponding to [wheelVelocities] and the given drive parameters.
     *
     * @param wheelVelocities wheel velocities (or wheel position deltas)
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     */
    @JvmStatic
    @JvmOverloads
    fun wheelToRobotVelocities(wheelVelocities: List<Double>, trackWidth: Double, wheelBase: Double = trackWidth): Pose2d {
        val k = (trackWidth + wheelBase) / 2.0
        val forwardKinematicsMatrix = MatrixUtils.createRealMatrix(
                arrayOf(
                        doubleArrayOf(0.25, 0.25, 0.25, 0.25),
                        doubleArrayOf(-0.25, 0.25, -0.25, 0.25),
                        doubleArrayOf(-0.25 / k, -0.25 / k, 0.25 / k, 0.25 / k)
                )
        )
        val wheelVelocityMatrix = MatrixUtils.createColumnRealMatrix(wheelVelocities.toDoubleArray())
        val poseVelocities = forwardKinematicsMatrix.multiply(wheelVelocityMatrix).transpose().data[0]
        return Pose2d(poseVelocities[0], poseVelocities[1], poseVelocities[2])
    }
}