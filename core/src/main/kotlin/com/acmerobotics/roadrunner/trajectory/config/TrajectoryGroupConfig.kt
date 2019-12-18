package com.acmerobotics.roadrunner.trajectory.config

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints
import com.fasterxml.jackson.annotation.JsonIgnore

class TrajectoryGroupConfig(
    var constraints: DriveConstraints,
    var distanceUnit: DistanceUnit,
    var driveType: DriveType,
    var trackWidth: Double?,
    var wheelBase: Double?,
    var lateralMultiplier: Double?
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
        @JsonIgnore get() = when (driveType) {
            DriveType.GENERIC -> constraints
            DriveType.MECANUM -> MecanumConstraints(constraints, trackWidth!!, wheelBase ?: trackWidth!!)
            DriveType.TANK -> TankConstraints(constraints, trackWidth!!)
        }
}
