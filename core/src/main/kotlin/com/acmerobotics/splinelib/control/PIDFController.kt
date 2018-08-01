package com.acmerobotics.splinelib.control

import kotlin.math.sign

/**
 * PID controller with various feedforward components. [kV], [kA], and [kStatic] are designed for DC motor feedforward
 * control (the most common kind of feedforward in FTC). [kF] provides a custom feedforward term for other plants.
 *
 * @param pid traditional PID coefficients
 * @param kV feedforward velocity gain
 * @param kA feedforward acceleration gain
 * @param kStatic constant additive feedforward contribution
 * @param kF custom, position-dependent feedforward (e.g., a cos term for arms)
 */
class PIDFController @JvmOverloads constructor(
        val pid: PIDCoefficients,
        val kV: Double = 0.0,
        val kA: Double = 0.0,
        val kStatic: Double = 0.0,
        val kF: (Double) -> Double = { 0.0 }
) {
    private var errorSum: Double = 0.0
    private var lastUpdateTimestamp: Double = Double.NaN

    /**
     * Target position (i.e., the controller setpoint)
     */
    var targetPosition: Double = 0.0

    /**
     * Run a single iteration of the controller.
     *
     * @param position current measured position (feedback)
     * @param velocity feedforward velocity
     * @param acceleration feedforward acceleration
     * @param currentTimestamp timestamp for the above parameters (intendend for simulation)
     */
    @JvmOverloads
    fun update(position: Double, velocity: Double = 0.0, acceleration: Double = 0.0, currentTimestamp: Double = System.nanoTime() / 1e9): Double {
        return if (lastUpdateTimestamp.isNaN()) {
            lastUpdateTimestamp = currentTimestamp
            0.0
        } else {
            val dt = currentTimestamp - lastUpdateTimestamp
            val error = targetPosition - position
            errorSum += error * dt
            val errorDeriv = error / dt

            val output = pid.kP * error + pid.kI * errorSum + pid.kD * (errorDeriv - velocity) + kV * velocity + kA * acceleration + kF(position)
            return output + sign(output) * kStatic
        }
    }

    /**
     * Reset the controller's integral sum.
     */
    fun reset() {
        errorSum = 0.0
        lastUpdateTimestamp = Double.NaN
    }
}
