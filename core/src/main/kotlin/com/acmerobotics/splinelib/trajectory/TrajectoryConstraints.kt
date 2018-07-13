package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Pose2d

interface TrajectoryConstraints {
    fun maximumVelocity(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double
    fun maximumAcceleration(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double
}