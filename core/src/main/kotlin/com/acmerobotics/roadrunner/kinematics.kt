package com.acmerobotics.roadrunner

import Twist2d

//fun fieldToRobotVelocity(fieldPose: Pose2d, fieldVel: Pose2d) =
//    Pose2d(fieldVel.vec().rotated(-fieldPose.heading), fieldVel.heading)

class WheelDeltas(
    val frontLeft: Double,
    val frontRight: Double,
    val backLeft: Double,
    val backRight: Double,
)

// TODO: how to integrate this with constraints?
class MecanumKinematics @JvmOverloads constructor(
    val trackWidth: Double,
    val lateralMultiplier: Double = 1.0
) {
    constructor(
        trackWidth: Double, wheelBase: Double,
        lateralMultiplier: Double = 1.0
    ): this((trackWidth + wheelBase) / 2, lateralMultiplier)

    // TODO: what about accel? maybe linear kinematics should be good?
    fun wheelDeltas(t: Twist2d) = WheelDeltas(
        t.x - lateralMultiplier * t.y - trackWidth * t.heading,
        t.x + lateralMultiplier * t.y - trackWidth * t.heading,
        t.x - lateralMultiplier * t.y + trackWidth * t.heading,
        t.x + lateralMultiplier * t.y + trackWidth * t.heading
    )

    fun twist(w: WheelDeltas) = Twist2d(
        (w.backLeft + w.frontRight + w.backRight + w.frontLeft) / 4,
        (w.backLeft + w.frontRight - w.frontLeft - w.backRight) / (4 * lateralMultiplier),
            (w.backRight + w.frontRight - w.frontLeft - w.backLeft) / (4 * trackWidth)
    )
}

// TODO: is a generic tracking wheel localizer worth it?
