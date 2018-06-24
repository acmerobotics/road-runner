package com.acmerobotics.library

class PathTrajectorySegment(val path: Path, val profile: MotionProfile) : TrajectorySegment {
    override fun duration() = profile.duration()

    override operator fun get(time: Double): Pose2d {
        return path.pose(profile[time].x)
    }

    override fun velocity(time: Double): Pose2d {
        val motionState = profile[time]
        return path.poseDeriv(motionState.x) * motionState.v
    }

    override fun acceleration(time: Double): Pose2d {
        val motionState = profile[time]
        return (path.poseSecondDeriv(motionState.x) * Math.pow(motionState.v, 2.0)) + (path.poseDeriv(motionState.x) * motionState.a)
    }
}