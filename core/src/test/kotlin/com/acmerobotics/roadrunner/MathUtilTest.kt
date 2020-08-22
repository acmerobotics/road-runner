package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.util.EPSILON
import com.acmerobotics.roadrunner.util.MathUtil.solveCubic
import com.acmerobotics.roadrunner.util.MathUtil.solveQuadratic
import org.junit.jupiter.api.Assertions.assertArrayEquals

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class MathUtilTest {
    private fun testQuadratic(a: Double, b: Double, c: Double, roots: DoubleArray) {
        assertArrayEquals(roots, solveQuadratic(a, b, c), EPSILON, "${a}x^2 + ${b}x + $c = 0")
    }

    private fun testCubic(a: Double, b: Double, c: Double, d: Double, roots: DoubleArray) {
        assertArrayEquals(roots, solveCubic(a, b, c, d), EPSILON, "${a}x^3 + ${b}x^2 + ${c}x + $d = 0")
    }

    @Test
    fun testQuadratics() {
        // normal case
        testQuadratic(1.0, 0.0, -1.0, doubleArrayOf(1.0, -1.0))

        // a = 0.0, simple fallback
        testQuadratic(0.0, 2.0, -1.0, doubleArrayOf(0.5))

        // discriminant = 0.0
        testQuadratic(1.0, -4.0, 4.0, doubleArrayOf(2.0))

        // discriminant < 0.0
        testQuadratic(1.0, 0.0, 1.0, doubleArrayOf())
    }

    @Test
    fun testCubics() {
        // 3 reals
        testCubic(1.0, -10.5, 32.0, -30.0, doubleArrayOf(6.0, 2.0, 2.5))

        // 1 real
        testCubic(1.0, -4.5, 17.0, -30.0, doubleArrayOf(2.5))

        // 2 unique reals
        testCubic(1.0, -10.0, 28.0, -24.0, doubleArrayOf(6.0, 2.0))

        // quadratic fallback
        testCubic(0.0, 1.0, 0.0, -1.0, doubleArrayOf(1.0, -1.0))
    }
}
