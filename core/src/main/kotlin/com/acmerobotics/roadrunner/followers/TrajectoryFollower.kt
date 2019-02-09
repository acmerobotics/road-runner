package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.util.NanoClock
import kotlin.math.abs

/**
 * Generic [Trajectory] follower for time-based pose reference tracking.
 *
 * @param admissibleError admissible/satisfactory pose error at the end of each move
 * @param timeout max time to wait for the error to be admissible
 * @param clock clock
 */
abstract class TrajectoryFollower @JvmOverloads constructor(
    private val admissibleError: Pose2d = Pose2d(),
    private val timeout: Double = 0.0,
    protected val clock: NanoClock = NanoClock.system()
) {
    private var startTimestamp: Double = 0.0
    private var admissible = false

    /**
     * Trajectory being followed if [isFollowing] is true.
     */
    var trajectory: Trajectory = Trajectory()
        protected set

    /**
     * Robot pose error computed in the last [update] call.
     */
    abstract var lastError: Pose2d
        protected set

    /**
     * Follow the given [trajectory].
     */
    open fun followTrajectory(trajectory: Trajectory) {
        this.startTimestamp = clock.seconds()
        this.trajectory = trajectory
        this.admissible = false
    }

    /**
     * Returns true if the current trajectory has finished executing.
     */
    fun isFollowing(): Boolean {
        val timeRemaining = trajectory.duration() - elapsedTime()
        return timeRemaining > 0 || (!admissible && timeRemaining > -timeout)
    }

    /**
     * Returns the elapsed time since the last [followTrajectory] call.
     */
    protected fun elapsedTime() = clock.seconds() - startTimestamp

    /**
     * Run a single iteration of the trajectory follower.
     *
     * @param currentPose current robot pose
     */
    fun update(currentPose: Pose2d): DriveSignal {
        val trajEndError = trajectory.end() - currentPose
        admissible = abs(trajEndError.x) < admissibleError.x
                && abs(trajEndError.y) < admissibleError.y
                && abs(trajEndError.heading) < admissibleError.heading
        return if (isFollowing()) {
            internalUpdate(currentPose)
        } else {
            DriveSignal()
        }
    }

    protected abstract fun internalUpdate(currentPose: Pose2d): DriveSignal
}