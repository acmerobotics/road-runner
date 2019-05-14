package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.*
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints
import com.acmerobotics.roadrunner.util.DoubleProgression

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

    /**
     * Generate a dynamic constraint trajectory.
     * @param path path
     * @param trajectoryConstraints trajectory constraints
     * @param start start motion state
     * @param goal goal motion state
     * @param resolution dynamic profile sampling resolution
     */
    @JvmOverloads
    fun generateTrajectory(
        path: Path,
        trajectoryConstraints: TrajectoryConstraints,
        start: MotionState = MotionState(0.0, 0.0, 0.0),
        goal: MotionState = MotionState(path.length(), 0.0, 0.0),
        resolution: Double = 0.25
    ) = Trajectory(path, generateProfile(path, trajectoryConstraints, start, goal, resolution))

    /**
     * Generate a simple constraint trajectory.
     * @param path path
     * @param driveConstraints drive constraints
     * @param start start motion state
     * @param goal goal motion state
     */
    @JvmOverloads
    fun generateSimpleTrajectory(
        path: Path,
        driveConstraints: DriveConstraints,
        start: MotionState = MotionState(0.0, 0.0, 0.0, 0.0),
        goal: MotionState = MotionState(path.length(), 0.0, 0.0, 0.0)
    ) = Trajectory(path, generateSimpleProfile(driveConstraints, start, goal))
}
