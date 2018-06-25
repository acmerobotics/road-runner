package com.acmerobotics.library

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import java.lang.Math.min
import java.lang.Math.pow


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class MotionProfileGeneratorTest {
    @Test
    fun testSimpleTriangle() {
        Graph.saveMotionProfile(
            "simpleTriangle",
            MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(0.0, 0.0, 0.0),
                MotionState(10.0, 0.0, 0.0),
                1000.0,
                5.0
            )
        )
    }

    @Test
    fun testSimpleTrap() {
        Graph.saveMotionProfile(
            "simpleTrap",
            MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(0.0, 0.0, 0.0),
                MotionState(10.0, 0.0, 0.0),
                5.0,
                5.0
            )
        )
    }

    @Test
    fun testSimpleTriangleStartingOffset() {
        Graph.saveMotionProfile(
            "simpleTriangleStartingOffset",
            MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(5.0, 0.0, 0.0),
                MotionState(15.0, 0.0, 0.0),
                1000.0,
                5.0
            )
        )
    }

    @Test
    fun testSimpleTriangleReversed() {
        Graph.saveMotionProfile(
            "simpleTriangleReversed",
            MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(10.0, 0.0, 0.0),
                MotionState(0.0, 0.0, 0.0),
                1000.0,
                5.0
            )
        )
    }

    @Test
    fun testSimpleTriangleStartingOffsetReversed() {
        Graph.saveMotionProfile(
            "simpleTriangleStartingOffsetReversed",
            MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(15.0, 0.0, 0.0),
                MotionState(5.0, 0.0, 0.0),
                1000.0,
                5.0
            )
        )
    }

    @Test
    fun testComplex() {
        Graph.saveMotionProfile(
            "complex",
            MotionProfileGenerator.generateMotionProfile(
                MotionState(0.0, 0.0, 0.0),
                MotionState(10.0, 0.0, 0.0),
                object : MotionConstraints {
                    override fun maximumVelocity(displacement: Double) = pow(displacement - 5.0, 4.0) + 1.0
                    override fun maximumAcceleration(displacement: Double) = 5.0
                }
            )
        )
    }

    @Test
    fun testComplex2() {
        Graph.saveMotionProfile(
            "complex2",
            MotionProfileGenerator.generateMotionProfile(
                MotionState(0.0, 0.0, 0.0),
                MotionState(10.0, 0.0, 0.0),
                object : MotionConstraints {
                    override fun maximumVelocity(displacement: Double) = pow(displacement - 5.0, 4.0) + 1.0
                    override fun maximumAcceleration(displacement: Double) = min(pow(displacement - 5.0, 4.0) + 1.0, 10.0)
                }
            )
        )
    }

    @Test
    fun testComplex2Reversed() {
        Graph.saveMotionProfile(
            "complex2Reversed",
            MotionProfileGenerator.generateMotionProfile(
                MotionState(10.0, 0.0, 0.0),
                MotionState(0.0, 0.0, 0.0),
                object : MotionConstraints {
                    override fun maximumVelocity(displacement: Double) = pow(displacement - 5.0, 4.0) + 1.0
                    override fun maximumAcceleration(displacement: Double) = min(pow(displacement - 5.0, 4.0) + 1.0, 10.0)
                }
            )
        )
    }
}