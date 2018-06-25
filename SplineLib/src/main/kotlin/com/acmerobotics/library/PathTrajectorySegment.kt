package com.acmerobotics.library

class PathTrajectorySegment(val path: HolonomicPath, resolution: Int = 250) : TrajectorySegment {
    val profile: MotionProfile

    init {
        val start = MotionState(0.0, 0.0, 0.0)
        val goal = MotionState(path.length(), 0.0, 0.0)
        profile = MotionProfileGenerator.generateMotionProfile(start, goal, DriveMotionConstraints(path), resolution)
    }

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