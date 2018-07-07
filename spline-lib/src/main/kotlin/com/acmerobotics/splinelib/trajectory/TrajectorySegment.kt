package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Pose2d

interface TrajectorySegment {
    fun duration(): Double
    operator fun get(time: Double): Pose2d
    fun velocity(time: Double): Pose2d
    fun acceleration(time: Double): Pose2d
}