package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints

/**
 * Builder for trajectories with *static* constraints.
 */
class SimpleTrajectoryBuilder(
    startPose: Pose2d,
    private val driveConstraints: DriveConstraints,
    private val start: MotionState = MotionState(0.0, 0.0, 0.0)
) : BaseTrajectoryBuilder(startPose) {

    override fun buildTrajectory(path: Path): Trajectory {
        val goal = MotionState(path.length(), 0.0, 0.0, 0.0)
        return TrajectoryGenerator.generateSimpleTrajectory(path, driveConstraints, start, goal)
    }
}
