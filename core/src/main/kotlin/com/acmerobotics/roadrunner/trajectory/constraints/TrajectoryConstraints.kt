package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.profile.SimpleMotionConstraints

/**
 * Trajectory-specific constraints for motion profile generation.
 */
interface TrajectoryConstraints {

    /**
     * Returns the maximum velocity and acceleration for the given pose derivatives.
     *
     * @param pose pose
     * @param poseDeriv pose derivative
     * @param poseSecondDeriv pose second derivative
     */
    operator fun get(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): SimpleMotionConstraints
}
