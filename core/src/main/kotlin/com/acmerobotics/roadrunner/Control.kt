package com.acmerobotics.roadrunner

import kotlin.math.withSign

// TODO: talk about units?
/**
 * @usesMathJax
 *
 * Kinematic motor feedforward
 *
 * @property[kS] kStatic, \(k_s\)
 * @property[kV] kVelocity, \(k_v\)
 * @property[kA] kStatic, \(k_a\)
 */
data class MotorFeedforward(
    @JvmField
    val kS: Double,
    @JvmField
    val kV: Double,
    @JvmField
    val kA: Double,
) {
    /**
     * @usesMathJax
     *
     * Computes the (normalized) voltage \(k_s \cdot \operatorname{sign}(k_v \cdot v + k_a \cdot a) + k_v \cdot v + k_a \cdot a\).
     *
     * @param[vel] \(v\)
     * @param[accel] \(a\)
     */
    fun compute(vel: Double, accel: Double): Double {
        val basePower = vel * kV + accel * kA
        return kS.withSign(basePower) + basePower
    }

    fun compute(vel: DualNum<Time>) = compute(vel[0], vel[1])
}

/**
 * Proportional position-velocity controller for a holonomic robot.
 */
class HolonomicController(
    @JvmField
    val axialPosGain: Double,
    @JvmField
    val lateralPosGain: Double,
    @JvmField
    val headingGain: Double,
    @JvmField
    val axialVelGain: Double,
    @JvmField
    val lateralVelGain: Double,
    @JvmField
    val headingVelGain: Double,
) {
    constructor(
        axialPosGain: Double,
        lateralPosGain: Double,
        headingGain: Double,
    ) : this(axialPosGain, lateralPosGain, headingGain, 0.0, 0.0, 0.0)

    /**
     * Computes the velocity and acceleration command. The frame `Target` is the reference robot, and the frame `Actual`
     * is the measured, physical robot.
     *
     * @return velocity command in the actual frame
     */
    fun compute(
        txWorldTarget: Transform2Dual<Time>,
        txWorldActual: Transform2,
        actualVelActual: Twist2,
    ): Twist2Dual<Time> {
        // pose error by a better name
        val txTargetActual = txWorldTarget.value().inverse() * txWorldActual

        val targetVelWorld = txWorldTarget.velocity()
        val txActualWorld = Transform2Dual.constant<Time>(txWorldActual.inverse(), 1)
        val targetVelActual = txActualWorld * targetVelWorld

        val velErrorActual = targetVelActual.value() - actualVelActual

        return targetVelActual +
            Twist2(
                Vector2(
                    txTargetActual.trans.x * axialPosGain,
                    txTargetActual.trans.y * lateralPosGain,
                ),
                txTargetActual.rot.log() * headingGain,
            ) +
            Twist2(
                Vector2(
                    velErrorActual.transVel.x * axialVelGain,
                    velErrorActual.transVel.y * lateralVelGain,
                ),
                velErrorActual.rotVel * headingVelGain,
            )
    }
}
