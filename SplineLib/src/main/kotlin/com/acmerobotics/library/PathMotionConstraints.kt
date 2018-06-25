package com.acmerobotics.library

interface PathMotionConstraints {
    fun maximumVelocity(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double
    fun maximumAcceleration(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double
}