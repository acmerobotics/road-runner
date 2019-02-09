package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d

data class DriveSignal @JvmOverloads constructor(val velocity: Pose2d = Pose2d(), val acceleration: Pose2d = Pose2d())
