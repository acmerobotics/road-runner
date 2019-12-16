package com.acmerobotics.roadrunner.trajectory.config

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints

class TrajectoryGroupConfig(
    val driveType: DriveType,
    val distanceUnit: DistanceUnit,
    val constraints: DriveConstraints,
    val trackWidth: Double,
    val wheelBase: Double?,
    val lateralMultiplier: Double?
) {
    enum class DistanceUnit {
        FOOT,
        INCH,
        METER,
        CENTIMETER,
        MILLIMETER
    }

    enum class DriveType {
        GENERIC,
        MECANUM,
        TANK
    }

    // TODO: incorporate the lateral multiplier
    val specificConstraints: DriveConstraints
        get() = when (driveType) {
            DriveType.GENERIC -> constraints
            DriveType.MECANUM -> MecanumConstraints(constraints, trackWidth, wheelBase!!)
            DriveType.TANK -> TankConstraints(constraints, trackWidth)
        }
}
