package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.util.DoubleProgression
import com.acmerobotics.roadrunner.util.EPSILON
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DoubleProgressionTest {
    @Test
    fun testNormalSplit() {
        val prog = DoubleProgression(0.0, 0.1, 10)

        val (firstHalf, secondHalf) = prog.split(0.25)
        assertEquals(3, firstHalf.size)
        assertEquals(7, secondHalf.size)
    }

    @Test
    fun testLowSplit() {
        val prog = DoubleProgression(0.0, 0.1, 10)

        val (firstHalf, secondHalf) = prog.split(-0.45)
        assertEquals(0, firstHalf.size)
        assertEquals(10, secondHalf.size)
    }

    @Test
    fun testHighSplit() {
        val prog = DoubleProgression(0.0, 0.1, 10)

        val (firstHalf, secondHalf) = prog.split(1.5)
        assertEquals(10, firstHalf.size)
        assertEquals(0, secondHalf.size)
    }

    @Test
    fun testIndexing() {
        val prog = DoubleProgression(0.0, 0.1, 10)

        assertEquals(2, prog.floorIndex(0.25))
        assertEquals(3, prog.ceilIndex(0.25))
        assertEquals(true, prog.contains(0.25))
        assertEquals(false, prog.contains(-0.55))
    }

    @Test
    fun testIterator() {
        val prog = DoubleProgression(0.0, 0.1, 10)

        assertEquals(0.9, prog.last(), EPSILON)
    }

    @Test
    fun testClosedIntervalConstruction() {
        val prog = DoubleProgression.fromClosedInterval(0.0, 1.0, 11)

        assertEquals(0.0, prog.first(), EPSILON)
        assertEquals(1.0, prog.last(), EPSILON)
    }
}
