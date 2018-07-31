package com.acmerobotics.splinelib.followers

import com.acmerobotics.splinelib.Pose2d
import com.acmerobotics.splinelib.trajectory.Trajectory

abstract class TrajectoryFollower {
    private var startTimestamp: Double = 0.0
    protected var trajectory: Trajectory = Trajectory()

    fun followTrajectory(trajectory: Trajectory, startTimestamp: Double = System.nanoTime() / 1e9) {
        this.startTimestamp = startTimestamp
        this.trajectory = trajectory
    }

    fun isFollowing(currentTimestamp: Double = System.nanoTime() / 1e9): Boolean {
        return elapsedTime(currentTimestamp) <= trajectory.duration()
    }

    fun elapsedTime(currentTimestamp: Double = System.nanoTime() / 1e9) = currentTimestamp - startTimestamp

    fun update(currentPose: Pose2d, currentTimestamp: Double = System.nanoTime() / 1e9): Boolean {
        return if (isFollowing(currentTimestamp)) {
            internalUpdate(currentPose, currentTimestamp)
            true
        } else {
            false
        }
    }

    fun awaitCompletion() {
        while (isFollowing() && !Thread.currentThread().isInterrupted) {
            try {
                Thread.sleep(5)
            } catch (e: InterruptedException) {
                Thread.currentThread().interrupt()
            }
        }
    }

    protected abstract fun internalUpdate(currentPose: Pose2d, currentTimestamp: Double)
}