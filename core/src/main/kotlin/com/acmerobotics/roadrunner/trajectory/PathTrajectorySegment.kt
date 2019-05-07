package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.*
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints
import com.acmerobotics.roadrunner.util.DoubleProgression
import kotlin.math.max
import kotlin.math.min

/**
 * Trajectory segment backed by a list of [Path] objects.
 *
 * @param paths paths
 * @param profile motion profile
 */
class PathTrajectorySegment(
    val paths: List<Path>,
    val profile: MotionProfile
) : TrajectorySegment {

    companion object {
        @Suppress("ComplexMethod")
        private fun generateProfile(
            paths: List<Path>,
            trajectoryConstraintsList: List<TrajectoryConstraints>,
            start: MotionState,
            goal: MotionState,
            resolution: Double
        ): MotionProfile {
            val length = paths.sumByDouble { it.length() }
            // TODO: make this a separate class
            val compositeConstraints = object : MotionConstraints() {
                override fun get(s: Double) = internalGet(s, reparam(s))

                override fun get(s: DoubleProgression) =
                    s.zip(internalReparam(s).asIterable()).map { internalGet(it.first, it.second) }

                private fun internalGet(s: Double, t: Double): SimpleMotionConstraints {
                    var remainingDisplacement = max(0.0, min(s, length))
                    for ((path, motionConstraints) in paths.zip(trajectoryConstraintsList)) {
                        if (remainingDisplacement <= path.length()) {
                            return motionConstraints[
                                path[remainingDisplacement, t],
                                path.deriv(remainingDisplacement, t),
                                path.secondDeriv(remainingDisplacement, t)
                            ]
                        }
                        remainingDisplacement -= path.length()
                    }
                    return trajectoryConstraintsList.last()[paths.last().end(),
                        paths.last().endDeriv(), paths.last().endSecondDeriv()]
                }

                private fun reparam(s: Double): Double {
                    var remainingDisplacement = s
                    for (i in paths.indices) {
                        val path = paths[i]
                        if (remainingDisplacement <= path.length()) {
                            return path.reparam(remainingDisplacement)
                        }
                        remainingDisplacement -= path.length()
                    }
                    return if (paths.last().reversed.last()) {
                        0.0
                    } else {
                        1.0
                    }
                }

                private fun internalReparam(s: DoubleProgression): DoubleArray {
                    val t = DoubleArray(s.items())
                    var offset = 0
                    var remainingDisplacement = s
                    for (i in paths.indices) {
                        val path = paths[i]
                        val splitDisplacement =
                            remainingDisplacement.split(path.length())
                        val segmentDisplacement = if (i == paths.lastIndex) {
                            remainingDisplacement
                        } else {
                            splitDisplacement.first
                        }
                        if (!segmentDisplacement.isEmpty()) {
                            path.reparam(segmentDisplacement).copyInto(t, offset, 0)
                            offset += segmentDisplacement.items()
                        }
                        remainingDisplacement = splitDisplacement.second - path.length()
                    }
                    return t
                }
            }

            return MotionProfileGenerator.generateMotionProfile(start, goal, compositeConstraints, resolution)
        }

        private fun generateSimpleProfile(
            constraints: DriveConstraints,
            start: MotionState,
            goal: MotionState
        ): MotionProfile {
            return MotionProfileGenerator.generateSimpleMotionProfile(start, goal,
                constraints.maxVel, constraints.maxAccel, constraints.maxJerk)
        }
    }

    /**
     * @param paths paths
     * @param trajectoryConstraintsList list of trajectory constraints
     * @param start profile start state
     * @param goal profile goal (end) state
     * @param resolution resolution for the motion profile
     */
    @JvmOverloads constructor(
        paths: List<Path> = emptyList(),
        trajectoryConstraintsList: List<TrajectoryConstraints> = emptyList(),
        start: MotionState = MotionState(0.0, 0.0, 0.0),
        goal: MotionState = MotionState(paths.sumByDouble { it.length() }, 0.0, 0.0),
        resolution: Double = 0.25
    ) : this(paths, generateProfile(paths, trajectoryConstraintsList, start, goal, resolution))

    /**
     * @param path path
     * @param trajectoryConstraints trajectory constraints
     * @param resolution resolution for the motion profile
     */
    @JvmOverloads constructor(
        path: Path,
        trajectoryConstraints: TrajectoryConstraints,
        start: MotionState = MotionState(0.0, 0.0, 0.0),
        goal: MotionState = MotionState(path.length(), 0.0, 0.0),
        resolution: Double = 0.25
    ) : this(listOf(path), listOf(trajectoryConstraints), start, goal, resolution)

    /**
     * @param paths paths
     * @param constraints drive constraints
     */
    @JvmOverloads constructor(
        paths: List<Path> = emptyList(),
        constraints: DriveConstraints,
        start: MotionState = MotionState(0.0, 0.0, 0.0, 0.0),
        goal: MotionState = MotionState(paths.sumByDouble { it.length() }, 0.0, 0.0, 0.0)
    ) : this(paths, generateSimpleProfile(constraints, start, goal))

    /**
     * @param path path
     * @param constraints drive constraints
     */
    @JvmOverloads constructor(
        path: Path,
        constraints: DriveConstraints,
        start: MotionState = MotionState(0.0, 0.0, 0.0, 0.0),
        goal: MotionState = MotionState(path.length(), 0.0, 0.0, 0.0)
    ) : this(listOf(path), generateSimpleProfile(constraints, start, goal))

    override fun duration() = profile.duration()

    override operator fun get(time: Double): Pose2d {
        var remainingDisplacement = profile[time].x
        for (path in paths) {
            if (remainingDisplacement <= path.length()) {
                return path[remainingDisplacement]
            }
            remainingDisplacement -= path.length()
        }
        return paths.lastOrNull()?.end() ?: Pose2d()
    }

    override fun velocity(time: Double): Pose2d {
        val motionState = profile[time]
        var remainingDisplacement = motionState.x
        for (path in paths) {
            if (remainingDisplacement <= path.length()) {
                return path.deriv(remainingDisplacement) * motionState.v
            }
            remainingDisplacement -= path.length()
        }
        return (paths.lastOrNull()?.endDeriv() ?: Pose2d()) * profile.end().v
    }

    override fun acceleration(time: Double): Pose2d {
        val motionState = profile[time]
        var remainingDisplacement = motionState.x
        for (path in paths) {
            if (remainingDisplacement <= path.length()) {
                return path.secondDeriv(remainingDisplacement) * motionState.v * motionState.v +
                        path.deriv(remainingDisplacement) * motionState.a
            }
            remainingDisplacement -= path.length()
        }
        return (paths.lastOrNull()?.endSecondDeriv() ?: Pose2d()) * profile.end().v * profile.end().v +
                (paths.lastOrNull()?.endDeriv() ?: Pose2d()) * profile.end().a
    }
}
