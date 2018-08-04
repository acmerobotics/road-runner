package com.acmerobotics.splinelib.followers

import com.acmerobotics.splinelib.Pose2d
import com.acmerobotics.splinelib.path.Path

/**
 * Generic [Path] follower for time-independent pose reference tracking.
 */
abstract class PathFollower {
    private var startTimestamp: Double = 0.0
    protected lateinit var path: Path

    /**
     * Follow the given [path] starting at [startTimestamp].
     *
     * @param path trajectory to follow
     * @param startTimestamp start timestamp override (often for simulation)
     */
    open fun followPath(path: Path, startTimestamp: Double = System.nanoTime() / 1e9) {
        this.startTimestamp = startTimestamp
        this.path = path
    }

    /**
     * Returns true if the current trajectory has finished executing.
     *
     * @param currentTimestamp current timestamp override (for simulation)
     * @return true if the trajectory has finished executing
     */
    abstract fun isFollowing(): Boolean

    /**
     * Run a single iteration of the trajectory follower.
     *
     * @param currentPose current robot pose
     * @param currentTimestamp current timestamp override (for simulation)
     */
    abstract fun update(currentPose: Pose2d, currentTimestamp: Double = System.nanoTime() / 1e9)
}