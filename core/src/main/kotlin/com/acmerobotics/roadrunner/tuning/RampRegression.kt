package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.roadrunner.util.MathUtil.numericalDerivative
import org.apache.commons.math3.stat.regression.SimpleRegression
import java.io.File
import kotlin.math.abs

/**
 * Container for ramp feedforward regression data.
 *
 * Here's the general procedure for gathering the requisite data:
 *   1. Slowly ramp the motor power/voltage and record encoder values along the way.
 *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 *      and an optional intercept (kStatic).
 *
 * @param timeSamples time samples
 * @param positionSamples position samples
 * @param powerSamples power (scaled voltage) samples
 */
class RampRegression @JvmOverloads constructor(
    private val timeSamples: MutableList<Double> = mutableListOf(),
    private val positionSamples: MutableList<Double> = mutableListOf(),
    private val powerSamples: MutableList<Double> = mutableListOf()
) {
    private val velSamples
        get() = numericalDerivative(timeSamples, positionSamples)

    /**
     * Feedforward parameter estimates from the ramp regression and additional summary statistics
     */
    data class RampResult(@JvmField val kV: Double, @JvmField val kStatic: Double, @JvmField val rSquare: Double)

    /**
     * Add a sample to the regression.
     */
    fun add(time: Double, position: Double, power: Double) {
        timeSamples.add(time)
        positionSamples.add(position)
        powerSamples.add(power)
    }

    /**
     * Fit data from a "ramp" test.
     * @param fitStatic true if kStatic should be fit (note: this affects the kV computation)
     */
    @JvmOverloads
    fun fit(fitStatic: Boolean = false): RampResult {
        val rampReg = SimpleRegression(fitStatic)
        velSamples.zip(powerSamples)
            .forEach { (vel, power) -> rampReg.addData(vel, power) }
        return RampResult(abs(rampReg.slope), abs(rampReg.intercept), rampReg.rSquare)
    }

    /**
     * Save the data to a CSV file for debugging or additional analysis.
     */
    fun save(file: File) {
        file.printWriter().use { out ->
            out.println("vel,power")
            velSamples.zip(powerSamples)
                .forEach { (vel, power) ->
                    out.println("$vel,$power")
                }
        }
    }
}
