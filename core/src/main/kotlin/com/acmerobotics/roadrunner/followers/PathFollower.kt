package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.util.NanoClock

/**
 * Generic [Path] follower for time-independent pose reference tracking.
 */
abstract class PathFollower @JvmOverloads constructor(protected val clock: NanoClock = NanoClock.default()) {
    private var startTimestamp: Double = 0.0
    protected lateinit var path: Path

    /**
     * Follow the given [path].
     */
    open fun followPath(path: Path) {
        this.startTimestamp = clock.seconds()
        this.path = path
    }

    /**
     * Returns true if the current trajectory has finished executing.
     */
    abstract fun isFollowing(): Boolean

    /**
     * Run a single iteration of the trajectory follower.
     *
     * @param currentPose current robot pose
     */
    abstract fun update(currentPose: Pose2d)
}