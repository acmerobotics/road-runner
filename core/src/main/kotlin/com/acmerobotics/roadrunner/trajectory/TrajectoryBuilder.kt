package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints

/**
 * Builder for trajectories with *dynamic* constraints.
 */
class TrajectoryBuilder(
    startPose: Pose2d,
    private val trajectoryConstraints: TrajectoryConstraints,
    private val start: MotionState = MotionState(0.0, 0.0, 0.0),
    private val resolution: Double = 0.25
) : BaseTrajectoryBuilder(startPose) {

    override fun buildTrajectory(path: Path): Trajectory {
        val goal = MotionState(path.length(), 0.0, 0.0)
        return TrajectoryGenerator.generateTrajectory(path, trajectoryConstraints, start, goal, resolution)
    }
}
