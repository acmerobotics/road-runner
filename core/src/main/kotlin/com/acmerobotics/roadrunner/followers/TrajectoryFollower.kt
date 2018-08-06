package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory

/**
 * Generic [Trajectory] follower for time-based pose reference tracking.
 */
abstract class TrajectoryFollower {
    private var startTimestamp: Double = 0.0
    protected var trajectory: Trajectory = Trajectory()

    /**
     * Follow the given [trajectory] starting at [startTimestamp].
     *
     * @param trajectory trajectory to follow
     * @param startTimestamp start timestamp override (often for simulation)
     */
    fun followTrajectory(trajectory: Trajectory, startTimestamp: Double = System.nanoTime() / 1e9) {
        this.startTimestamp = startTimestamp
        this.trajectory = trajectory
    }

    /**
     * Returns true if the current trajectory has finished executing.
     *
     * @param currentTimestamp current timestamp override (for simulation)
     * @return true if the trajectory has finished executing
     */
    fun isFollowing(currentTimestamp: Double = System.nanoTime() / 1e9): Boolean {
        return elapsedTime(currentTimestamp) <= trajectory.duration()
    }

    /**
     * Returns the elapsed time since [startTimestamp].
     *
     * @param currentTimestamp current timestamp override (for simulation)
     * @return elapsed time
     */
    protected fun elapsedTime(currentTimestamp: Double = System.nanoTime() / 1e9) = currentTimestamp - startTimestamp

    /**
     * Run a single iteration of the trajectory follower.
     *
     * @param currentPose current robot pose
     * @param currentTimestamp current timestamp override (for simulation)
     */
    abstract fun update(currentPose: Pose2d, currentTimestamp: Double = System.nanoTime() / 1e9)
}