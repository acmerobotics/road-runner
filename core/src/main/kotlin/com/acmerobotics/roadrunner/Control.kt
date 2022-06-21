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
     * Computes the velocity and acceleration command.
     *
     * @param[actualBodyVel] actual velocity in the actual body frame
     * @return velocity command in the actual body frame
     */
    fun compute(
        targetPose: Transform2Dual<Time>,
        actualPose: Transform2,
        actualBodyVel: Twist2,
    ): Twist2Dual<Time> {
        val poseError = poseError(targetPose.value(), actualPose)

        val targetVel = targetPose.velocity()
        val velError = targetVel.value() - actualBodyVel

        // TODO: is inverseThenTimes() better than this?
        return targetPose.rotation.inverse() * targetVel +
            Twist2(
                Vector2(
                    poseError.transError.x * axialPosGain,
                    poseError.transError.y * lateralPosGain,
                ),
                poseError.rotError * headingGain,
            ) +
            Twist2(
                Vector2(
                    velError.transVel.x * axialVelGain,
                    velError.transVel.y * lateralVelGain,
                ),
                velError.rotVel * headingVelGain,
            )
    }
}
