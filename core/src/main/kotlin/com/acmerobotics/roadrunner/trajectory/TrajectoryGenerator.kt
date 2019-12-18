package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.*
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints
import com.acmerobotics.roadrunner.util.DoubleProgression
import com.acmerobotics.roadrunner.util.epsilonEquals

/**
 * Trajectory generator for creating trajectories with dynamic and static constraints from paths.
 */
object TrajectoryGenerator {

    private fun generateProfile(
        path: Path,
        constraints: TrajectoryConstraints,
        start: MotionState,
        goal: MotionState,
        resolution: Double
    ): MotionProfile {
        return MotionProfileGenerator.generateMotionProfile(start, goal, object : MotionConstraints() {
            override fun get(s: Double): SimpleMotionConstraints {
                val t = path.reparam(s)
                return constraints[
                    path[s, t],
                    path.deriv(s, t),
                    path.secondDeriv(s, t)
                ]
            }

            override fun get(s: DoubleProgression) =
                s.zip(path.reparam(s).asIterable())
                    .map { (s, t) ->
                        constraints[
                            path[s, t],
                            path.deriv(s, t),
                            path.secondDeriv(s, t)
                        ]
                    }
        }, resolution)
    }

    private fun generateSimpleProfile(
        constraints: DriveConstraints,
        start: MotionState,
        goal: MotionState
    ): MotionProfile {
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal,
            constraints.maxVel, constraints.maxAccel, constraints.maxJerk)
    }

    private fun pointToTime(path: Path, profile: MotionProfile, point: Vector2d): Double {
        val distToStart = path.start().vec() distTo point
        val distToEnd = path.end().vec() distTo point
        val s0 = path.length() * distToStart / (distToStart + distToEnd)
        val s = path.project(point, s0)
        var tLo = 0.0
        var tHi = profile.duration()
        while (!(tLo epsilonEquals tHi)) {
            val tMid = 0.5 * (tLo + tHi)
            if (profile[tMid].x > s) {
                tHi = tMid
            } else {
                tLo = tMid
            }
        }
        return 0.5 * (tLo + tHi)
    }

    private fun mergeMarkers(
        path: Path,
        profile: MotionProfile,
        temporalMarkers: List<RelativeTemporalMarker>,
        spatialMarkers: List<SpatialMarker>
    ): List<AbsoluteTemporalMarker> {
        return temporalMarkers.map { (time, callback) ->
            AbsoluteTemporalMarker(time(profile.duration()), callback) } +
            spatialMarkers.map { (point, callback) ->
                AbsoluteTemporalMarker(pointToTime(path, profile, point), callback) }
    }

    /**
     * Generate a dynamic constraint trajectory.
     * @param path path
     * @param constraints trajectory constraints
     * @param start start motion state
     * @param goal goal motion state
     * @param temporalMarkers temporal markers
     * @param spatialMarkers spatial markers
     * @param resolution dynamic profile sampling resolution
     */
    @Suppress("LongParameterList")
    @JvmOverloads
    fun generateTrajectory(
        path: Path,
        constraints: TrajectoryConstraints,
        start: MotionState = MotionState(0.0, 0.0, 0.0),
        goal: MotionState = MotionState(path.length(), 0.0, 0.0),
        temporalMarkers: List<RelativeTemporalMarker> = emptyList(),
        spatialMarkers: List<SpatialMarker> = emptyList(),
        resolution: Double = 0.25
    ): Trajectory {
        val profile = generateProfile(path, constraints, start, goal, resolution)
        val markers = mergeMarkers(path, profile, temporalMarkers, spatialMarkers)
        return Trajectory(path, profile, markers)
    }

    /**
     * Generate a simple constraint trajectory.
     * @param path path
     * @param constraints drive constraints
     * @param start start motion state
     * @param goal goal motion state
     * @param temporalMarkers temporal markers
     * @param spatialMarkers spatial markers
     */
    @Suppress("LongParameterList")
    @JvmOverloads
    fun generateSimpleTrajectory(
        path: Path,
        constraints: DriveConstraints,
        start: MotionState = MotionState(0.0, 0.0, 0.0, 0.0),
        goal: MotionState = MotionState(path.length(), 0.0, 0.0, 0.0),
        temporalMarkers: List<RelativeTemporalMarker> = emptyList(),
        spatialMarkers: List<SpatialMarker> = emptyList()
    ): Trajectory {
        val profile = generateSimpleProfile(constraints, start, goal)
        val markers = mergeMarkers(path, profile, temporalMarkers, spatialMarkers)
        return Trajectory(path, profile, markers)
    }
}
