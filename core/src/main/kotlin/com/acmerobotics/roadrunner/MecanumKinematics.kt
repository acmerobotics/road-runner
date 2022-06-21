package com.acmerobotics.roadrunner

import kotlin.math.abs

/**
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
 * @param[lateralMultiplier] factor that multiplies strafe velocity to compensate for slip; increase it to boost the
 * distance traveled in the strafe direction
 */
data class MecanumKinematics @JvmOverloads constructor(
    @JvmField
    val trackWidth: Double,
    @JvmField
    val lateralMultiplier: Double = 1.0
) {
    /**
     * @param[wheelBase] distance between wheels on the same side; see the diagram in [MecanumKinematics]
     */
    constructor(
        trackWidth: Double,
        wheelbase: Double,
        lateralMultiplier: Double = 1.0
    ) : this((trackWidth + wheelbase) / 2, lateralMultiplier)

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

    // TODO: is forward the best name?
    // TODO: test forward, inverse composition
    fun <Param> forward(w: WheelIncrements<Param>) = Twist2IncrementDual(
        Vector2Dual(
            (w.backLeft + w.frontRight + w.backRight + w.frontLeft) * 0.25,
            (w.backLeft + w.frontRight - w.frontLeft - w.backRight) * (0.25 / lateralMultiplier),
        ),
        (w.backRight + w.frontRight - w.frontLeft - w.backLeft) * (0.25 / trackWidth),
    )

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
        constructor(vels: List<DualNum<Param>>) : this(vels[0], vels[1], vels[2], vels[3])

        fun all() = listOf(frontLeft, frontRight, backLeft, backRight)
    }

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
