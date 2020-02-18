package com.acmerobotics.roadrunner.trajectory.config

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints
import com.fasterxml.jackson.annotation.JsonIgnore

data class TrajectoryGroupConfig(
    val maxVel: Double,
    val maxAccel: Double,
    val maxAngVel: Double,
    val maxAngAccel: Double,
    val robotLength: Double,
    val robotWidth: Double,
    val driveType: DriveType,
    val trackWidth: Double?,
    val wheelBase: Double?,
    val lateralMultiplier: Double?
) {
    enum class DriveType {
        GENERIC,
        MECANUM,
        TANK
    }

    @JsonIgnore private val baseConstraints = DriveConstraints(
        maxVel = maxVel,
        maxAccel = maxAccel,
        maxAngVel = maxAngVel,
        maxAngAccel = maxAngAccel,
        maxJerk = 0.0,
        maxAngJerk = 0.0
    )

    @JsonIgnore val constraints = when (driveType) {
        DriveType.GENERIC -> baseConstraints
        DriveType.MECANUM -> MecanumConstraints(baseConstraints, trackWidth!!, wheelBase ?: trackWidth, lateralMultiplier!!)
        DriveType.TANK -> TankConstraints(baseConstraints, trackWidth!!)
    }
}
