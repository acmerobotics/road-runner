package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.SimpleMotionConstraints

/**
 * Trajectory-specific constraints for motion profile generation.
 */
interface TrajectoryConstraints {

    /**
     * Returns the maximum velocity and acceleration for the given pose derivatives.
     *
     * @param pose pose
     * @param deriv pose derivative
     * @param secondDeriv pose second derivative
     */
    operator fun get(pose: Pose2d, deriv: Pose2d, secondDeriv: Pose2d): SimpleMotionConstraints
}
