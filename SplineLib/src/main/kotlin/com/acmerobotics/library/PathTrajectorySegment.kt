package com.acmerobotics.library

class PathTrajectorySegment(val path: HolonomicPath, val profile: MotionProfile) : TrajectorySegment {
    override fun duration() = profile.duration()

    override operator fun get(time: Double): Pose2d {
        return path[profile[time].x]
    }

    override fun velocity(time: Double): Pose2d {
        val motionState = profile[time]
        return path.deriv(motionState.x) * motionState.v
    }

    override fun acceleration(time: Double): Pose2d {
        val motionState = profile[time]
        return (path.secondDeriv(motionState.x) * Math.pow(motionState.v, 2.0)) + (path.deriv(motionState.x) * motionState.a)
    }
}