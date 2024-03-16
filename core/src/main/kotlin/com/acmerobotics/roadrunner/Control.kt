package com.acmerobotics.roadrunner

import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.withSign

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
    fun compute(vel: Double, accel: Double) = kS.withSign(vel) + kV * vel + kA * accel

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
        targetPose: Pose2dDual<Time>,
        actualPose: Pose2d,
        actualVelActual: PoseVelocity2d,
    ): PoseVelocity2dDual<Time> {
        // TODO: Are these names useful for anyone else?
        val targetVelWorld = targetPose.velocity()
        val txTargetWorld = Pose2dDual.constant<Time>(targetPose.value().inverse(), 2)
        val targetVelTarget = txTargetWorld * targetVelWorld

        val velErrorActual = targetVelTarget.value() - actualVelActual

        val error = targetPose.value().minusExp(actualPose)
        return targetVelTarget +
            PoseVelocity2d(
                Vector2d(
                    axialPosGain * error.position.x,
                    lateralPosGain * error.position.y,
                ),
                headingGain * error.heading.log(),
            ) +
            PoseVelocity2d(
                Vector2d(
                    axialVelGain * velErrorActual.linearVel.x,
                    lateralVelGain * velErrorActual.linearVel.y,
                ),
                headingVelGain * velErrorActual.angVel,
            )
    }
}

/**
 * @usesMathJax
 *
 * Ramsete controller for tracking tank trajectories.
 *
 * The standard Ramsete control law from equation \((5.12)\) of [this paper](https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf) is
 * \[
 *   \begin{pmatrix}v\\ \omega\end{pmatrix} =
 *   \begin{pmatrix}
 *     v_d \cos (\theta_d - \theta) + 2 \zeta \sqrt{\omega_d^2 + b v_d^2} \Big\lbrack (x_d - x) \cos \theta + (y_d - y) \sin \theta \Big\rbrack\\
 *     \omega_d + b v_d \frac{\sin(\theta_d - \theta)}{\theta_d - \theta} \Big\lbrack (x_d - x) \cos \theta - (y_d - y) \sin \theta \Big\rbrack + 2 \zeta \sqrt{\omega_d^2 + b v_d^2} (\theta_d - \theta)
 *   \end{pmatrix}
 * \]
 * where \(\zeta \in (0, 1)\) and \(b \gt 0\). Since \(b\) has units, we substitute \(b = \frac{\bar{b}}{l^2}\) where
 * \(l\) is the track width.
 */
// defaults taken from https://github.com/wpilibsuite/allwpilib/blob/3fdb2f767d466e00d19e487fdb64d33c22ccc7d5/wpimath/src/main/native/cpp/controller/RamseteController.cpp#L31-L33
// with a track width of 1 meter
class RamseteController @JvmOverloads constructor(
    @JvmField
    val trackWidth: Double,
    @JvmField
    val zeta: Double = 0.7,
    @JvmField
    val bBar: Double = 2.0,
) {
    @JvmField
    val b = bBar / (trackWidth * trackWidth)

    /**
     * Computes the velocity and acceleration command. The frame `Target` is the reference robot, and the frame `Actual`
     * is the measured, physical robot.
     *
     * @return velocity command in the actual frame
     */
    fun compute(
        s: DualNum<Time>,
        targetPose: Pose2dDual<Arclength>,
        actualPose: Pose2d,
    ): PoseVelocity2dDual<Time> {
        val targetHeading = targetPose.heading.value()
        val tangentHeading = targetPose.position.drop(1).value().angleCast()
        val dir = tangentHeading.real * targetHeading.real + tangentHeading.imag * targetHeading.imag
        val vRef = dir * s[1]

        val omegaRef = targetPose.reparam(s).heading.velocity()[0]

        val k = 2.0 * zeta * sqrt(omegaRef * omegaRef + b * vRef * vRef)

        fun sinc(x: Double): Double {
            val u = x + snz(x)
            return sin(u) / u
        }

        val error = targetPose.value().minusExp(actualPose)
        return PoseVelocity2dDual.constant(
            PoseVelocity2d(
                Vector2d(
                    vRef * error.heading.real + k * error.position.x,
                    0.0
                ),
                omegaRef + k * error.heading.log() + b * vRef * sinc(error.heading.log()) * error.position.y,
            ),
            2
        )
    }
}
