package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.TestUtil.assertContinuous
import com.acmerobotics.roadrunner.profile.*
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import java.lang.Math.min
import java.lang.Math.pow

const val RESOLUTION = 1000

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class MotionProfileGeneratorTest {

    /**
     * Verifies continuity and start/goal satisfaction and saves an image of the profile for manual inspection.
     */
    private fun testProfile(name: String, start: MotionState, goal: MotionState, profile: MotionProfile, verifyAccel: Boolean = false) {
        // save it
        GraphUtil.saveMotionProfile("profiles/$name", profile)

        // verify start state satisfaction
        assertEquals(start.x, profile.start().x, 1e-4)
        assertEquals(start.v, profile.start().v, 1e-4)
        if (verifyAccel) {
            assertEquals(start.a, profile.start().a, 1e-4)
        }

        // verify goal state satisfaction
        assertEquals(goal.x, profile.end().x, 1e-4)
        assertEquals(goal.v, profile.end().v, 1e-4)
        if (verifyAccel) {
            assertEquals(goal.a, profile.end().a, 1e-4)
        }

        // verify continuity
        val t = (0..RESOLUTION).map { it * profile.duration() / RESOLUTION }
        assertContinuous(t.map { profile[it].x }, 1.0)
        assertContinuous(t.map { profile[it].v }, 1.0)
        if (verifyAccel) {
            assertContinuous(t.map { profile[it].a }, 1.0)
        }
    }

    // simple (i.e., constant max vel/accel) accel-limited tests

    @Test
    fun testSimpleTriangle() {
        testProfile(
            "simpleTriangle",
            MotionState(0.0, 0.0, 0.0),
            MotionState(10.0, 0.0, 0.0),
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
        testProfile(
            "simpleTrap",
            MotionState(0.0, 0.0, 0.0),
            MotionState(10.0, 0.0, 0.0),
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
        testProfile(
            "simpleTriangleStartingOffset",
            MotionState(5.0, 0.0, 0.0),
            MotionState(15.0, 0.0, 0.0),
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
        testProfile(
            "simpleTriangleReversed",
            MotionState(10.0, 0.0, 0.0),
            MotionState(0.0, 0.0, 0.0),
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
        testProfile(
            "simpleTriangleStartingOffsetReversed",
            MotionState(15.0, 0.0, 0.0),
            MotionState(5.0, 0.0, 0.0),
            MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(15.0, 0.0, 0.0),
                MotionState(5.0, 0.0, 0.0),
                1000.0,
                5.0
            )
        )
    }

    @Test
    fun testSimpleConstraintViolation() {
        testProfile(
                "simpleConstraintViolation",
                MotionState(0.0, 60.0, 0.0),
                MotionState(10.0, 0.0, 0.0),
                MotionProfileGenerator.generateSimpleMotionProfile(
                        MotionState(0.0, 60.0, 0.0),
                        MotionState(10.0, 0.0, 0.0),
                        1000.0,
                        5.0
                )
        )
    }

    // complex (i.e., time-varying constraints) accel-limited tests

    @Test
    fun testComplex() {
        testProfile(
            "complex",
            MotionState(0.0, 0.0, 0.0),
            MotionState(10.0, 0.0, 0.0),
            MotionProfileGenerator.generateMotionProfile(
                MotionState(0.0, 0.0, 0.0),
                MotionState(10.0, 0.0, 0.0),
                object : MotionConstraints() {
                    override fun get(s: Double) = SimpleMotionConstraints(
                        pow(s - 5.0, 4.0) + 1.0, 5.0
                    )
                }
            )
        )
    }

    @Test
    fun testComplex2() {
        testProfile(
            "complex2",
            MotionState(0.0, 0.0, 0.0),
            MotionState(10.0, 0.0, 0.0),
            MotionProfileGenerator.generateMotionProfile(
                MotionState(0.0, 0.0, 0.0),
                MotionState(10.0, 0.0, 0.0),
                object : MotionConstraints() {
                    override fun get(s: Double) = SimpleMotionConstraints(
                        pow(s - 5.0, 4.0) + 1.0,
                        min(pow(s - 5.0, 4.0) + 1.0, 10.0)
                    )
                }
            )
        )
    }

    @Test
    fun testComplex2Reversed() {
        testProfile(
            "complex2Reversed",
            MotionState(10.0, 0.0, 0.0),
            MotionState(0.0, 0.0, 0.0),
            MotionProfileGenerator.generateMotionProfile(
                MotionState(10.0, 0.0, 0.0),
                MotionState(0.0, 0.0, 0.0),
                object : MotionConstraints() {
                    override fun get(s: Double) = SimpleMotionConstraints(
                        pow(s - 5.0, 4.0) + 1.0,
                        min(pow(s - 5.0, 4.0) + 1.0, 10.0)
                    )
                }
            )
        )
    }

    // simple jerk-limited tests

    @Test
    fun testJerkLimited() {
        testProfile(
                "jerkLimited",
                MotionState(0.0, 50.0, -25.0),
                MotionState(100.0, -5.0, 20.0),
                MotionProfileGenerator.generateSimpleMotionProfile(
                        MotionState(0.0, 50.0, -25.0),
                        MotionState(100.0, -5.0, 20.0),
                        15.0,
                        30.0,
                        30.0
                ),
                true
        )
    }

    @Test
    fun testShortJerkLimited() {
        testProfile(
                "jlShort",
                MotionState(0.0, 0.0, 0.0),
                MotionState(10.0, 0.0, 0.0),
                MotionProfileGenerator.generateSimpleMotionProfile(
                        MotionState(0.0, 0.0, 0.0),
                        MotionState(10.0, 0.0, 0.0),
                        15.0,
                        30.0,
                        30.0
                ),
                true
        )
    }

    @Test
    fun testLongJerkLimited() {
        testProfile(
                "jlLong",
                MotionState(0.0, 0.0, 0.0),
                MotionState(100.0, 0.0, 0.0),
                MotionProfileGenerator.generateSimpleMotionProfile(
                        MotionState(0.0, 0.0, 0.0),
                        MotionState(100.0, 0.0, 0.0),
                        15.0,
                        30.0,
                        30.0
                ),
                true
        )
    }

    @Test
    fun testJLConstraintViolations() {
        testProfile(
                "jlConstraintViolations",
                MotionState(0.0, 10.0),
                MotionState(1.0, 0.0),
                MotionProfileGenerator.generateSimpleMotionProfile(
                        MotionState(0.0, 10.0),
                        MotionState(1.0, 0.0),
                        15.0,
                        30.0,
                        40.0
                )
        )
    }
}
