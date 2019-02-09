package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.*
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints
import com.acmerobotics.roadrunner.util.DoubleProgression
import kotlin.math.max
import kotlin.math.min

/**
 * Trajectory segment backed by a list of [Path] objects.
 *
 * @param paths paths
 * @param trajectoryConstraintsList motion constraints for each respective path
 * @param resolution resolution used for the motion profile (see [MotionProfileGenerator.generateMotionProfile])
 */
class PathTrajectorySegment @JvmOverloads constructor(
    val paths: List<Path> = emptyList(),
    val trajectoryConstraintsList: List<TrajectoryConstraints> = emptyList(),
    resolution: Double = 0.25
) : TrajectorySegment {
    /**
     * @param path path
     * @param trajectoryConstraints trajectory constraints
     * @param resolution resolution for the motion profile
     */
    @JvmOverloads constructor(
        path: Path,
        trajectoryConstraints: TrajectoryConstraints,
        resolution: Double = 0.25
    ) : this(listOf(path), listOf(trajectoryConstraints), resolution)

    /**
     * Motion profile used for time parametrization of the paths.
     */
    val profile: MotionProfile

    init {
        val length = paths.sumByDouble { it.length() }
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

        val start = MotionState(0.0, 0.0, 0.0)
        val goal = MotionState(length, 0.0, 0.0)
        profile = MotionProfileGenerator.generateMotionProfile(start, goal, compositeConstraints, resolution)
    }

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