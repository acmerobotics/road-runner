package com.acmerobotics.roadrunner

data class DriveSignal @JvmOverloads constructor(val velocity: Pose2d = Pose2d(), val acceleration: Pose2d = Pose2d())