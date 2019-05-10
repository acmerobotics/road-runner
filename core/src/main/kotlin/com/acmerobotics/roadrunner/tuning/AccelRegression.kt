package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.util.MathUtil.numericalDerivative
import org.apache.commons.math3.stat.regression.SimpleRegression
import java.io.File
import kotlin.math.abs

/**
 * Container for acceleration feedforward regression data. This data can be gathered through pretty much any motion
 * although it's standard to apply constant/step voltage/power.
 *
 * @param timeSamples time samples
 * @param positionSamples position samples
 * @param powerSamples power (scaled voltage) samples
 */
class AccelRegression @JvmOverloads constructor(
    private val timeSamples: MutableList<Double> = mutableListOf(),
    private val positionSamples: MutableList<Double> = mutableListOf(),
    private val powerSamples: MutableList<Double> = mutableListOf()
) {
    private val velSamples
        get() = numericalDerivative(timeSamples, positionSamples)

    private val accelSamples
        get() = numericalDerivative(timeSamples, velSamples)

    /**
     * Feedforward parameter estimates from the ramp regression and additional summary statistics
     */
    data class AccelResult(@JvmField val kA: Double, @JvmField val rSquare: Double)

    /**
     * Add a sample to the regression.
     */
    fun add(time: Double, position: Double, power: Double) {
        timeSamples.add(time)
        positionSamples.add(position)
        powerSamples.add(power)
    }

    /**
     * Fit kA from the accel test data.
     */
    fun fit(kV: Double, kStatic: Double): AccelResult {
        val accelReg = SimpleRegression(false)
        velSamples.zip(powerSamples)
            .map { (vel, power) ->
                val predPower = Kinematics.calculateMotorFeedforward(
                    vel, 0.0,
                    kV, 0.0, kStatic
                )
                power - predPower
            }
            .zip(accelSamples)
            .forEach { (accelPower, accel) ->
                accelReg.addData(accel, accelPower)
            }
        return AccelResult(abs(accelReg.slope), accelReg.rSquare)
    }

    /**
     * Save the data to a CSV file for debugging or additional analysis.
     */
    fun save(file: File) {
        file.printWriter().use { out ->
            out.println("vel,accel,power")
            velSamples.zip(accelSamples)
                .zip(powerSamples)
                .forEach { (pair, power) ->
                    val (vel, accel) = pair
                    out.println("$vel,$accel,$power")
                }
        }
    }
}
