package com.acmerobotics.splinelib.followers

import com.acmerobotics.splinelib.Pose2d
import com.acmerobotics.splinelib.trajectory.Trajectory

/**
 * Generic trajectory follower for time-based pose reference tracking.
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
     * Run a single step of the trajectory follower. If the trajectory has finished (based on [currentTimestamp]), this
     * does nothing.
     *
     * @param currentPose current robot pose
     * @param currentTimestamp current timestamp override (for simulation)
     * @return true if the update was carried out, false if the trajectory already finished
     */
    fun update(currentPose: Pose2d, currentTimestamp: Double = System.nanoTime() / 1e9): Boolean {
        return if (isFollowing(currentTimestamp)) {
            internalUpdate(currentPose, currentTimestamp)
            true
        } else {
            false
        }
    }

    /**
     * Wait for the current trajectory to finish executing.
     */
    fun awaitCompletion() {
        while (isFollowing() && !Thread.currentThread().isInterrupted) {
            try {
                Thread.sleep(5)
            } catch (e: InterruptedException) {
                Thread.currentThread().interrupt()
            }
        }
    }

    /**
     * Internal version of [update].
     *
     * @param currentPose current pose
     * @param currentTimestamp current timestamp
     */
    protected abstract fun internalUpdate(currentPose: Pose2d, currentTimestamp: Double)
}