package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.path.LineSegment
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.util.NanoClock

/**
 * Generic [Path] follower for time-independent pose reference tracking.
 *
 * @param clock clock
 */
abstract class PathFollower @JvmOverloads constructor(protected val clock: NanoClock = NanoClock.system()) {
    private var startTimestamp: Double = 0.0

    /**
     * Path being followed if [isFollowing] is true.
     */
    var path: Path = Path()
        protected set

    /**
     * Robot pose error computed in the last [update] call.
     */
    abstract var lastError: Pose2d
        protected set

    /**
     * Follow the given [path].
     */
    open fun followPath(path: Path) {
        this.startTimestamp = clock.seconds()
        this.path = path
    }

    /**
     * Returns true if the current path has finished executing.
     */
    abstract fun isFollowing(): Boolean

    /**
     * Run a single iteration of the path follower.
     *
     * @param currentPose current robot pose
     */
    abstract fun update(currentPose: Pose2d)
}