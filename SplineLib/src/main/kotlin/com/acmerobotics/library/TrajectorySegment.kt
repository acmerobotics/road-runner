package com.acmerobotics.library

interface TrajectorySegment {
    fun duration(): Double
    operator fun get(time: Double): Pose2d
    fun velocity(time: Double): Pose2d
    fun acceleration(time: Double): Pose2d
}