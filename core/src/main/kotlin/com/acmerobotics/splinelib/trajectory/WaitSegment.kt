package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Pose2d

class WaitSegment(private val pose: Pose2d, private val duration: Double) : TrajectorySegment {
    override fun duration() = duration

    override fun get(time: Double) = pose

    override fun velocity(time: Double) = Pose2d(0.0, 0.0, 0.0)

    override fun acceleration(time: Double) = Pose2d(0.0, 0.0, 0.0)
}