package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.SimpleMotionConstraints

/**
 * Trajectory-specific constraints for motion profile generation.
 */
interface TrajectoryConstraints {

    /**
     * Returns the maximum velocity and acceleration for the given path displacement and pose derivatives.
     *
     * @param s path displacement
     * @param pose pose
     * @param deriv pose derivative
     * @param secondDeriv pose second derivative
     */
    operator fun get(s: Double, pose: Pose2d, deriv: Pose2d, secondDeriv: Pose2d): SimpleMotionConstraints
}
