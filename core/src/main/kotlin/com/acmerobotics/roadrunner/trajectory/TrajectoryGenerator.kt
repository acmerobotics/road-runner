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
        trajectoryConstraints: TrajectoryConstraints,
        start: MotionState,
        goal: MotionState,
        resolution: Double
    ): MotionProfile {
        return MotionProfileGenerator.generateMotionProfile(start, goal, object : MotionConstraints() {
            override fun get(s: Double): SimpleMotionConstraints {
                val t = path.reparam(s)
                return trajectoryConstraints[
                    path[s, t],
                    path.deriv(s, t),
                    path.secondDeriv(s, t)
                ]
            }

            override fun get(s: DoubleProgression) =
                s.zip(path.reparam(s).asIterable())
                    .map { (s, t) ->
                        trajectoryConstraints[
                            path[s, t],
                            path.deriv(s, t),
                            path.secondDeriv(s, t)
                        ]
                    }
        }, resolution)
    }

    private fun generateSimpleProfile(
        driveConstraints: DriveConstraints,
        start: MotionState,
        goal: MotionState
    ): MotionProfile {
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal,
            driveConstraints.maxVel, driveConstraints.maxAccel, driveConstraints.maxJerk)
    }

    private fun pointToTime(path: Path, profile: MotionProfile, point: Vector2d): Double {
        val distToStart = path.start().pos() distanceTo point
        val distToEnd = path.end().pos() distanceTo point
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
        temporalMarkers: List<TemporalMarker>,
        spatialMarkers: List<SpatialMarker>
    ): List<TemporalMarker> {
        return temporalMarkers + spatialMarkers.map {
                (point, callback) -> TemporalMarker(pointToTime(path, profile, point), callback) }
    }

    /**
     * Generate a dynamic constraint trajectory.
     * @param path path
     * @param trajectoryConstraints trajectory constraints
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
        trajectoryConstraints: TrajectoryConstraints,
        start: MotionState = MotionState(0.0, 0.0, 0.0),
        goal: MotionState = MotionState(path.length(), 0.0, 0.0),
        temporalMarkers: List<TemporalMarker> = emptyList(),
        spatialMarkers: List<SpatialMarker> = emptyList(),
        resolution: Double = 0.25
    ): Trajectory {
        val profile = generateProfile(path, trajectoryConstraints, start, goal, resolution)
        val markers = mergeMarkers(path, profile, temporalMarkers, spatialMarkers)
        return Trajectory(path, profile, markers)
    }

    /**
     * Generate a simple constraint trajectory.
     * @param path path
     * @param driveConstraints drive constraints
     * @param start start motion state
     * @param goal goal motion state
     * @param temporalMarkers temporal markers
     * @param spatialMarkers spatial markers
     */
    @Suppress("LongParameterList")
    @JvmOverloads
    fun generateSimpleTrajectory(
        path: Path,
        driveConstraints: DriveConstraints,
        start: MotionState = MotionState(0.0, 0.0, 0.0, 0.0),
        goal: MotionState = MotionState(path.length(), 0.0, 0.0, 0.0),
        temporalMarkers: List<TemporalMarker> = emptyList(),
        spatialMarkers: List<SpatialMarker> = emptyList()
    ): Trajectory {
        val profile = generateSimpleProfile(driveConstraints, start, goal)
        val markers = mergeMarkers(path, profile, temporalMarkers, spatialMarkers)
        return Trajectory(path, profile, markers)
    }
}
