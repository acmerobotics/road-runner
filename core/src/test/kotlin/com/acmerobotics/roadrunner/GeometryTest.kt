package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.random.Random
import kotlin.test.assertEquals

class GeometryTest {
    @Test
    fun testRotationExpLog() {
        repeat(100) {
            val x = Random.Default.nextDouble(-PI, PI)
            assertEquals(x, Rotation2d.exp(x).log(), 1e-6)
        }
    }

    @Test
    fun testRotationInverseInverse() {
        repeat(100) {
            val x = Random.Default.nextDouble(-PI, PI)
            assertEquals(x, Rotation2d.exp(x).inverse().inverse().log(), 1e-6)
        }
    }

    @Test
    fun testTransformExpLog() {
        repeat(100) {
            val incrExp = Twist2dIncr(
                Vector2d(
                    Random.Default.nextDouble(),
                    Random.Default.nextDouble(),
                ),
                Random.Default.nextDouble(),
            )
            val incrActual = Pose2d.exp(incrExp).log()
            assertEquals(incrExp.transIncr.x, incrActual.transIncr.x, 1e-6)
            assertEquals(incrExp.transIncr.y, incrActual.transIncr.y, 1e-6)
            assertEquals(incrExp.rotIncr, incrActual.rotIncr, 1e-6)
        }
    }

    @Test
    fun testTransformInverseInverse() {
        repeat(100) {
            val incrExp = Twist2dIncr(
                Vector2d(
                    Random.Default.nextDouble(),
                    Random.Default.nextDouble(),
                ),
                Random.Default.nextDouble(),
            )
            val incrActual = Pose2d.exp(incrExp).inverse().inverse().log()
            assertEquals(incrExp.transIncr.x, incrActual.transIncr.x, 1e-6)
            assertEquals(incrExp.transIncr.y, incrActual.transIncr.y, 1e-6)
            assertEquals(incrExp.rotIncr, incrActual.rotIncr, 1e-6)
        }
    }

    @Test
    fun testLocalErrorExample() {
        val target = Pose2d(Vector2d(1.0, 2.0), Rotation2d.exp(0.0))
        val actual = Pose2d(Vector2d(1.0, 1.0), Rotation2d.exp(PI / 2))
        val e = target.minusExp(actual)

        assertEquals(1.0, e.trans.x, 1e-6)
        assertEquals(0.0, e.trans.y, 1e-6)
        assertEquals(-PI / 2, e.rot.log(), 1e-6)
    }
}
