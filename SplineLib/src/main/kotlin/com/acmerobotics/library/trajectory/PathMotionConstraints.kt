package com.acmerobotics.library.trajectory

import com.acmerobotics.library.Pose2d

interface PathMotionConstraints {
    fun maximumVelocity(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double
    fun maximumAcceleration(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double
}