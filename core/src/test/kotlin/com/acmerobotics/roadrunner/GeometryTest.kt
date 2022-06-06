package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.sqrt
import kotlin.random.Random
import kotlin.test.assertEquals

class GeometryTest {
    @Test
    fun testRotationExp() {
        repeat (100) {
            val x = Random.Default.nextDouble(-PI, PI)
            assertEquals(x, Rotation2.exp(x).log(), 1e-6)
        }
    }

    @Test
    fun testTransformExp() {
        repeat (100) {
            val incrExp = Twist2Incr(
                Vector2(
                    Random.Default.nextDouble(),
                    Random.Default.nextDouble(),
                ),
                Random.Default.nextDouble(),
            )
            val incrActual = Transform2.exp(incrExp).log()
            assertEquals(incrExp.transIncr.x, incrActual.transIncr.x, 1e-6)
            assertEquals(incrExp.transIncr.y, incrActual.transIncr.y, 1e-6)
            assertEquals(incrExp.rotIncr, incrActual.rotIncr, 1e-6)
        }
    }

    @Test
    fun testLocalError() {
        val target = Transform2(Vector2(0.0, 1.0), Rotation2.exp(0.0))
        val actual = Transform2(Vector2(0.0, 0.0), Rotation2.exp(PI / 2))
        val e = localError(target, actual)
        assertEquals(1.0, e.transError.x, 1e-6)
        assertEquals(0.0, e.transError.y, 1e-6)
        assertEquals(-PI / 2, e.rotError, 1e-6)
    }
}
