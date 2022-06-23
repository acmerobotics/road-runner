package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.random.Random
import kotlin.test.assertEquals

class GeometryTest {
    @Test
    fun testRotationExp() {
        repeat(100) {
            val x = Random.Default.nextDouble(-PI, PI)
            assertEquals(x, Rotation2.exp(x).log(), 1e-6)
        }
    }

    @Test
    fun testTransformExp() {
        repeat(100) {
            val incrExp = Twist2Increment(
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
        val target = Transform2(Vector2(1.0, 2.0), Rotation2.exp(0.0))
        val actual = Transform2(Vector2(1.0, 1.0), Rotation2.exp(PI / 2))
        val e = target.minusExp(actual)

        assertEquals(1.0, e.trans.x, 1e-6)
        assertEquals(0.0, e.trans.y, 1e-6)
        assertEquals(-PI / 2, e.rot.log(), 1e-6)
    }

    @Test
    fun testPoseErrorReplacement() {
        data class Transform2Error(@JvmField val transError: Vector2, @JvmField val rotError: Double)

        fun poseError(targetPose: Transform2, actualPose: Transform2): Transform2Error {
            val transErrorWorld = targetPose.trans - actualPose.trans
            val rotError = targetPose.rot - actualPose.rot
            return Transform2Error(actualPose.rot.inverse() * transErrorWorld, rotError)
        }

        val r = Random.Default
        repeat(100) {
            val target = Transform2(Vector2(r.nextDouble(), r.nextDouble()), Rotation2.exp(2 * PI * r.nextDouble()))
            val actual = Transform2(Vector2(r.nextDouble(), r.nextDouble()), Rotation2.exp(2 * PI * r.nextDouble()))
            val e1 = poseError(target, actual)
            val e2 = target.minusExp(actual)

            assertEquals(e1.transError.x, e2.trans.x, 1e-6)
            assertEquals(e1.transError.y, e2.trans.y, 1e-6)
            assert(abs(Rotation2.exp(e1.rotError) - e2.rot) < 1e-6)
        }
    }
}
