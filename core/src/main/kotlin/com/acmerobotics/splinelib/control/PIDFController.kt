package com.acmerobotics.splinelib.control

import kotlin.math.sign

/**
 * This class implements a PID controller with various feedforward components.
 */
class PIDFController(val pid: PIDCoefficients, val kV: Double = 0.0, val kA: Double = 0.0, val kStatic: Double = 0.0, val kF: (Double) -> Double = { 0.0 }) {
    private var errorSum: Double = 0.0
    private var lastUpdateTimestamp: Double = Double.NaN
    var targetPosition: Double = 0.0

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

    fun reset() {
        errorSum = 0.0
        lastUpdateTimestamp = Double.NaN
    }
}
