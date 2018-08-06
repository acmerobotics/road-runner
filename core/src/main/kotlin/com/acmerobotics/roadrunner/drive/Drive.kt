package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d

/**
 * Abstraction for generic drivetrain motion and localization.
 */
abstract class Drive {
    /**
     * The robot's current pose estimate.
     */
    var poseEstimate: Pose2d = Pose2d()

    /**
     * Sets the [poseVelocity] of the robot.
     */
    abstract fun setVelocity(poseVelocity: Pose2d)

    /**
     * Updates [poseEstimate] with the most recent positional change.
     */
    abstract fun updatePoseEstimate()
}