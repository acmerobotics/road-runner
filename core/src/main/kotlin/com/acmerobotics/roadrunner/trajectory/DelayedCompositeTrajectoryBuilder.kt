package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints

open class DelayedCompositeTrajectoryBuilder @JvmOverloads constructor(
        getStartPose: () -> Pose2d,
        globalConstraints: DriveConstraints,
        resolution: Int = 250
) : DelayedTrajectoryBuilder(
        getStartPose,
        globalConstraints,
        resolution
) {
    init {
        beginComposite()
    }

    override fun build(): Trajectory {
        closeComposite()
        return super.build()
    }
}