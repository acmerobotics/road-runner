package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints

/**
 * Easy-to-use builder for creating composite [Trajectory] instances.
 *
 * @param startPose start pose
 * @param globalConstraints global drive constraints (overridable for specific segments)
 * @param resolution resolution used for path-based segments (see [PathTrajectorySegment])
 */

class CompositeTrajectoryBuilder @JvmOverloads constructor(
        startPose: Pose2d,
        private val globalConstraints: DriveConstraints,
        private val resolution: Int = 250
) : TrajectoryBuilder(startPose, globalConstraints, resolution){
    init {
        beginComposite()
    }

    override fun build(): Trajectory {
        closeComposite()
        return super.build()
    }
}