package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d

/**
 * Abstraction for generic robot drive motion and localization. Robot poses are specified in a coordinate system with
 * positive x pointing forward, positive y pointing left, and positive heading measured counter-clockwise from the
 * x-axis.
 */
abstract class Drive {
    /**
     * Localizer used to determine the evolution of [poseEstimate].
     */
    abstract var localizer: Localizer

    /**
     * The robot's current pose estimate.
     */
    var poseEstimate: Pose2d
        get() = localizer.poseEstimate
        set(value) {
            localizer.poseEstimate = value
        }

    /**
     * Updates [poseEstimate] with the most recent positional change.
     */
    fun updatePoseEstimate() {
        localizer.update()
    }

    /**
     * Sets the [poseVelocity] of the robot.
     */
    abstract fun setVelocity(poseVelocity: Pose2d)
}