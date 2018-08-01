package com.acmerobotics.splinelib.drive

import com.acmerobotics.splinelib.Pose2d

interface Drive {
    fun setVelocity(poseVelocity: Pose2d)

    fun getPoseEstimate(): Pose2d
    fun resetPoseEstimate(newPose: Pose2d)
    fun updatePoseEstimate(timestamp: Double = System.nanoTime() / 1e9)
}