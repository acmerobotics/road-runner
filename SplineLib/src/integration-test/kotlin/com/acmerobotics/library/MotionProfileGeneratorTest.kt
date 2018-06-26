package com.acmerobotics.library

import com.acmerobotics.library.profile.MotionConstraints
import com.acmerobotics.library.profile.MotionProfileGenerator
import com.acmerobotics.library.profile.MotionState
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import java.lang.Math.min
import java.lang.Math.pow


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class MotionProfileGeneratorTest {
    @Test
    fun testSimpleTriangle() {
        GraphUtil.saveMotionProfile(
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
        GraphUtil.saveMotionProfile(
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
        GraphUtil.saveMotionProfile(
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
        GraphUtil.saveMotionProfile(
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
        GraphUtil.saveMotionProfile(
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
        GraphUtil.saveMotionProfile(
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
        GraphUtil.saveMotionProfile(
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
        GraphUtil.saveMotionProfile(
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