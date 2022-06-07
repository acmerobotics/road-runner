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
        val target = Transform2(Vector2(1.0, 2.0), Rotation2.exp(0.0))
        val actual = Transform2(Vector2(1.0, 1.0), Rotation2.exp(PI / 2))
        val e = localError(target, actual)
        assertEquals(1.0, e.transError.x, 1e-6)
        assertEquals(0.0, e.transError.y, 1e-6)
        assertEquals(-PI / 2, e.rotError, 1e-6)
    }

//    @Test
//    fun testKinematicsExpr() {
//        val txWorldRobot = Transform2Dual<Time>(
//            Vector2Dual(
//                DualNum(doubleArrayOf(1.0, 2.0, 3.0)),
//                DualNum(doubleArrayOf(4.0, 5.0, 6.0)),
//            ),
//            Rotation2Dual.exp(
//                DualNum(doubleArrayOf(7.0, 8.0, 9.0))
//            )
//        )
//
//        val derivWorld = txWorldRobot.velocity().constant()
//        val derivRobot = txWorldRobot.constant().inverse() * derivWorld
//        println(Twist2Dual.constant<Time>(derivRobot, 1))
//
//        println(txWorldRobot.constant().rotation.inverse() * derivWorld.transVel)
//
//        println(txWorldRobot.inverse().velocity())
//    }
}
