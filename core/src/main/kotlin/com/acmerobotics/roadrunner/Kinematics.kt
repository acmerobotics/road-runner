package com.acmerobotics.roadrunner

import kotlin.math.abs

data class WheelVelocities<Param>(
    @JvmField
    val frontLeft: DualNum<Param>,
    @JvmField
    val frontRight: DualNum<Param>,
    @JvmField
    val backLeft: DualNum<Param>,
    @JvmField
    val backRight: DualNum<Param>,
) {
    fun all() = listOf(frontLeft, frontRight, backLeft, backRight)
}

data class WheelIncrements<Param>(
    @JvmField
    val frontLeft: DualNum<Param>,
    @JvmField
    val frontRight: DualNum<Param>,
    @JvmField
    val backLeft: DualNum<Param>,
    @JvmField
    val backRight: DualNum<Param>,
)

data class MecanumKinematics @JvmOverloads constructor(
    @JvmField
    val trackWidth: Double,
    @JvmField
    val lateralMultiplier: Double = 1.0
) {
    constructor(
        trackWidth: Double,
        wheelBase: Double,
        lateralMultiplier: Double = 1.0
    ) : this((trackWidth + wheelBase) / 2, lateralMultiplier)

    // TODO: test forward, inverse composition
    fun <Param> forward(w: WheelIncrements<Param>) = Twist2IncrementDual(
        Vector2Dual(
            (w.backLeft + w.frontRight + w.backRight + w.frontLeft) * 0.25,
            (w.backLeft + w.frontRight - w.frontLeft - w.backRight) * (0.25 / lateralMultiplier),
        ),
        (w.backRight + w.frontRight - w.frontLeft - w.backLeft) * (0.25 / trackWidth),
    )

    fun <Param> inverse(t: Twist2Dual<Param>) = WheelVelocities(
        t.transVel.x - t.transVel.y * lateralMultiplier - t.rotVel * trackWidth,
        t.transVel.x + t.transVel.y * lateralMultiplier + t.rotVel * trackWidth,
        t.transVel.x + t.transVel.y * lateralMultiplier - t.rotVel * trackWidth,
        t.transVel.x - t.transVel.y * lateralMultiplier + t.rotVel * trackWidth,
    )

    inner class MaxWheelVelocityConstraint(@JvmField val maxWheelVel: Double) : VelocityConstraint {
        override fun maxRobotVel(robotPose: Transform2Dual<Arclength>) =
            inverse(robotPose.inverse() * robotPose.velocity())
                .all()
                .minOf { abs(maxWheelVel / it[0]) }
    }
}
