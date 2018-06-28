package com.acmerobotics.library.trajectory

import com.acmerobotics.library.Pose2d
import com.acmerobotics.library.path.Path
import com.acmerobotics.library.profile.MotionConstraints
import com.acmerobotics.library.profile.MotionProfile
import com.acmerobotics.library.profile.MotionProfileGenerator
import com.acmerobotics.library.profile.MotionState
import kotlin.math.max
import kotlin.math.min

class PathTrajectorySegment(
    val paths: List<Path>,
    val motionConstraintsList: List<PathMotionConstraints>,
    resolution: Int = 250
) : TrajectorySegment {
    val profile: MotionProfile

    init {
        val length = paths.sumByDouble { it.length() }
        val compositeConstraints = object : MotionConstraints {
            override fun maximumVelocity(displacement: Double): Double {
                var remainingDisplacement = max(0.0, min(displacement, length))
                for ((path, motionConstraints) in paths.zip(motionConstraintsList)) {
                    if (remainingDisplacement <= path.length()) {
                        return motionConstraints.maximumVelocity(
                            path[remainingDisplacement],
                            path.deriv(remainingDisplacement),
                            path.secondDeriv(remainingDisplacement)
                        )
                    }
                    remainingDisplacement -= path.length()
                }
                return motionConstraintsList.last()
                    .maximumVelocity(paths.last().end(), paths.last().endDeriv(), paths.last().endSecondDeriv())
            }

            override fun maximumAcceleration(displacement: Double): Double {
                var remainingDisplacement = max(0.0, min(displacement, length))
                for ((path, motionConstraints) in paths.zip(motionConstraintsList)) {
                    if (remainingDisplacement <= path.length()) {
                        return motionConstraints.maximumAcceleration(
                            path[remainingDisplacement],
                            path.deriv(remainingDisplacement),
                            path.secondDeriv(remainingDisplacement)
                        )
                    }
                    remainingDisplacement -= path.length()
                }
                return motionConstraintsList.last()
                    .maximumAcceleration(paths.last().end(), paths.last().endDeriv(), paths.last().endSecondDeriv())
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
        return paths.last().end()
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
        return paths.last().endDeriv() * profile.end().v
    }

    override fun acceleration(time: Double): Pose2d {
        val motionState = profile[time]
        var remainingDisplacement = motionState.x
        for (path in paths) {
            if (remainingDisplacement <= path.length()) {
                return path.secondDeriv(remainingDisplacement) * motionState.v * motionState.v + path.deriv(
                    remainingDisplacement
                ) * motionState.a
            }
            remainingDisplacement -= path.length()
        }
        return paths.last().endSecondDeriv() * profile.end().v * profile.end().v + paths.last().endDeriv() * profile.end().a
    }
}