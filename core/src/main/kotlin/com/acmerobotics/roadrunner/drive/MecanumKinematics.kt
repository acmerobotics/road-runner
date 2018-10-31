package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d

/**
 * Mecanum drive kinematic equations. All wheel positions and velocities are given starting with front left and
 * proceeding counter-clockwise (i.e., front left, rear left, rear right, front right). Robot poses are specified in a
 * coordinate system with positive x pointing forward, positive y pointing left, and positive heading measured
 * counter-clockwise from the x-axis.
 *
 * [This paper](http://www.chiefdelphi.com/media/papers/download/2722) provides a motivated derivation.
 */
object MecanumKinematics {

    /**
     * Computes the wheel velocities corresponding to [robotPoseVelocity] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotPoseVelocity velocity of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     */
    @JvmStatic
    @JvmOverloads
    fun robotToWheelVelocities(robotPoseVelocity: Pose2d, trackWidth: Double, wheelBase: Double = trackWidth): List<Double> {
        val k = (trackWidth + wheelBase) / 2.0
        return listOf(
                robotPoseVelocity.x - robotPoseVelocity.y - k * robotPoseVelocity.heading,
                robotPoseVelocity.x + robotPoseVelocity.y - k * robotPoseVelocity.heading,
                robotPoseVelocity.x - robotPoseVelocity.y + k * robotPoseVelocity.heading,
                robotPoseVelocity.x + robotPoseVelocity.y + k * robotPoseVelocity.heading
        )
    }

    /**
     * Computes the wheel accelerations corresponding to [robotPoseAcceleration] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotPoseAcceleration acceleration of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     */
    @JvmStatic
    @JvmOverloads
    // follows from linearity of the derivative
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
        return Pose2d(
                wheelVelocities.sum(),
                wheelVelocities[1] + wheelVelocities[3] - wheelVelocities[0] - wheelVelocities[2],
                (wheelVelocities[2] + wheelVelocities[3] - wheelVelocities[0] - wheelVelocities[1]) / k
        ) * 0.25
    }
}