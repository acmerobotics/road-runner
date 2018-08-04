package com.acmerobotics.splinelib.drive

import com.acmerobotics.splinelib.Pose2d

// TODO: add some kind of FF support (i.e., kV, kA, and kStatic) at this level
// or even the next level down (TankDrive, MecanumDrive)
interface Drive {
    fun setVelocity(poseVelocity: Pose2d)

    fun getPoseEstimate(): Pose2d
    fun resetPoseEstimate(newPose: Pose2d)
    fun updatePoseEstimate(timestamp: Double = System.nanoTime() / 1e9)
}