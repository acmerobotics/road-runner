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

    @Test
    fun testLerpLookup() {
        assertEquals(0.0, lerpLookup(listOf(0.0, 2.0), listOf(0.0, 3.0), 0.0))
        assertEquals(3.0, lerpLookup(listOf(0.0, 2.0), listOf(0.0, 3.0), 2.0))
        assertEquals(3.5, lerpLookup(listOf(1.0, 2.0), listOf(3.0, 4.0), 1.5))

        assertEquals(0.0, lerpLookup(listOf(0.0, 2.0), listOf(0.0, 3.0), -1.0))
        assertEquals(3.0, lerpLookup(listOf(0.0, 2.0), listOf(0.0, 3.0), 4.0))
    }

    @Test
    fun testLerpLookupMap() {
        repeat(50) {
            val source = List(20) { (Math.random() - 0.5) * 100 }.sorted()
            val target = List(20) { (Math.random() - 0.5) * 100 }.sorted()
            val queries = List(20) { (Math.random() - 0.5) * 200 }.sorted()

            assertEquals(
                queries.map { lerpLookup(source, target, it) },
                lerpLookupMap(source, target, queries)
            )
        }
    }
}
