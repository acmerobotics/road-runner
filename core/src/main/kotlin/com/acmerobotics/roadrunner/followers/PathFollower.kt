package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.NanoClock
import kotlin.math.abs

/**
 * Generic [Path] follower for time-independent pose reference tracking.
 *
 * @param clock clock
 */
abstract class PathFollower @JvmOverloads constructor(
    private val admissibleError: Pose2d = Pose2d(),
    protected val clock: NanoClock = NanoClock.system()
) {
    private var startTimestamp: Double = 0.0
    private var admissible = false
    internal var executedFinalUpdate = false

    /**
     * Path being followed if [isFollowing] is true.
     */
    lateinit var path: Path
        protected set

    /**
     * Robot pose error computed in the last [update] call.
     */
    abstract var lastError: Pose2d
        protected set

    /**
     * The projected displacement along the path found in the last [update] call (may be initialized to Double.NaN.)
     */
    abstract var displacement: Double
        protected set

    /**
     * Follow the given [path].
     *
     * @param path path
     * @param
     */
    open fun followPath(path: Path) {
        this.startTimestamp = clock.seconds()
        this.path = path
        this.admissible = false

        executedFinalUpdate = false
    }

    private fun internalIsFollowing() = !admissible

    /**
     * Returns true if the current path is currently executing.
     */
    fun isFollowing() = !executedFinalUpdate && internalIsFollowing()

    /**
     * Run a single iteration of the path follower.
     *
     * @param currentPose current robot pose
     */
    @JvmOverloads
    fun update(currentPose: Pose2d, currentRobotVel: Pose2d? = null): DriveSignal {
        val pathEndError = path.end() - currentPose + Pose2d()
        admissible = abs(pathEndError.x) < admissibleError.x &&
                abs(pathEndError.y) < admissibleError.y &&
                abs(Angle.normDelta(pathEndError.heading)) < admissibleError.heading
        return if (internalIsFollowing() || executedFinalUpdate) {
            internalUpdate(currentPose, currentRobotVel)
        } else {
            executedFinalUpdate = true
            DriveSignal()
        }
    }

    protected abstract fun internalUpdate(currentPose: Pose2d, currentRobotVel: Pose2d?): DriveSignal
}
