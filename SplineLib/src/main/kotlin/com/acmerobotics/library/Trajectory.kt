package com.acmerobotics.library

class Trajectory(val segments: MutableList<TrajectorySegment> = mutableListOf()) {
    fun duration() = segments.map { it.duration() }.sum()

    operator fun get(time: Double): Pose2d {
        var remainingTime = time
        for (segment in segments) {
            if (remainingTime <= segment.duration()) {
                return segment[remainingTime]
            }
            remainingTime -= segment.duration()
        }
        throw RuntimeException() // TODO
    }

    fun velocity(time: Double): Pose2d {
        var remainingTime = time
        for (segment in segments) {
            if (remainingTime <= segment.duration()) {
                return segment.velocity(remainingTime)
            }
            remainingTime -= segment.duration()
        }
        throw RuntimeException() // TODO
    }

    fun acceleration(time: Double): Pose2d {
        var remainingTime = time
        for (segment in segments) {
            if (remainingTime <= segment.duration()) {
                return segment.acceleration(remainingTime)
            }
            remainingTime -= segment.duration()
        }
        throw RuntimeException() // TODO
    }
}