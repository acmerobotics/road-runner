package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import kotlin.math.sqrt
import kotlin.test.assertEquals

class MathTest {
    @Test
    fun testIntegralScan() {
        // examples taken from the Gander and Gautschi paper
        assertEquals(2.0 / 3.0, integralScan(0.0, 1.0, 1e-12) { sqrt(it) }.sums.last(), 1e-11)
        assert(integralScan(0.0, 1.0, 1e-12) { 1.0 / sqrt(1.0 - it * it) }.sums.last().isNaN())
        assertEquals(
            7.5,
            integralScan(0.0, 5.0, 1e-6) {
                when {
                    it < 1 -> it + 1
                    it < 3 -> 3 - it
                    else -> 2.0
                }
            }.sums.last(),
            1e-4
        )
    }
}
