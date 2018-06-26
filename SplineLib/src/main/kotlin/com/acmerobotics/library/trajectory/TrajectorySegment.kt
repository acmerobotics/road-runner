package com.acmerobotics.library.trajectory

import com.acmerobotics.library.Pose2d

interface TrajectorySegment {
    fun duration(): Double
    operator fun get(time: Double): Pose2d
    fun velocity(time: Double): Pose2d
    fun acceleration(time: Double): Pose2d
}