package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.profile.MotionConstraints
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import java.lang.Math.min
import java.lang.Math.pow


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class MotionProfileGeneratorTest {
    @Test
    fun testSimpleTriangle() {
        GraphUtil.saveMotionProfile(
            "profiles/simpleTriangle",
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
            "profiles/simpleTrap",
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
            "profiles/simpleTriangleStartingOffset",
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
            "profiles/simpleTriangleReversed",
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
            "profiles/simpleTriangleStartingOffsetReversed",
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
            "profiles/complex",
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
            "profiles/complex2",
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
            "profiles/complex2Reversed",
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

    // TODO: add more jerk-limited test cases
    @Test
    fun testJerkLimited() {
        GraphUtil.saveMotionProfile(
                "profiles/jerkLimited",
                MotionProfileGenerator.generateSimpleMotionProfile(
                        MotionState(0.0, 50.0, -25.0),
                        MotionState(100.0, -5.0, 20.0),
                        15.0,
                        30.0,
                        30.0
                )
        )
    }

    @Test
    fun testConstraintViolations() {
        GraphUtil.saveMotionProfile(
                "profiles/constraintViolations",
                MotionProfileGenerator.generateSimpleMotionProfile(
                        MotionState(0.0, 10.0),
                        MotionState(1.0, 0.0),
                        15.0,
                        30.0
                )
        )
    }
}