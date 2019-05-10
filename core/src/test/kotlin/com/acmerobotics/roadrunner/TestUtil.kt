package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.util.MathUtil.numericalDerivative
import kotlin.math.abs
import org.assertj.core.api.Assertions.assertThat

object TestUtil {
    fun assertDerivEquals(x: List<Double>, dx: List<Double>, ds: Double, epsilon: Double, errorFreq: Double = 0.01) {
        val numDx = numericalDerivative(x, ds)
        val count = dx.zip(numDx)
                .map { abs(it.first - it.second) }
                .filter { it > epsilon }
                .count()
        val freq = count.toDouble() / x.size
        assertThat(freq).isLessThanOrEqualTo(errorFreq)
    }

    fun assertContinuous(values: List<Double>, epsilon: Double) {
        assertThat(values.drop(1)
                .zip(values.dropLast(1))
                .map { it.first - it.second }
                .max() ?: 0.0).isLessThan(epsilon)
    }
}
