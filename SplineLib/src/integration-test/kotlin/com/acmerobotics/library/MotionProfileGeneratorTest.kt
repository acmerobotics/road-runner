package com.acmerobotics.library

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import org.knowm.xchart.BitmapEncoder
import org.knowm.xchart.BitmapEncoder.BitmapFormat
import org.knowm.xchart.QuickChart
import java.io.File
import java.lang.Math.min
import java.lang.Math.pow


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class MotionProfileGeneratorTest {
    companion object {
        private const val GRAPH_DIR = "./graphs/"
        private const val GRAPH_DPI = 300

        fun saveMotionProfileGraph(name: String, profile: MotionProfile, includeAcceleration: Boolean = true, resolution: Int = 1000) {
            val timeData = (0..resolution).map { it / resolution.toDouble() * profile.duration() }.toDoubleArray()
            val positionData = timeData.map { profile[it].x }.toDoubleArray()
            val velocityData = timeData.map { profile[it].v }.toDoubleArray()

            val labels = mutableListOf("x(t)", "v(t)")
            val data = mutableListOf(positionData, velocityData)

            if (includeAcceleration) {
                val accelerationData = timeData.map { profile[it].a }.toDoubleArray()

                labels.add("a(t)")
                data.add(accelerationData)
            }

            val graph = QuickChart.getChart(
                name,
                "time (sec)",
                "",
                labels.toTypedArray(),
                timeData,
                data.toTypedArray()
            )

            File(GRAPH_DIR).mkdirs()

            BitmapEncoder.saveBitmapWithDPI(graph, "$GRAPH_DIR$name", BitmapFormat.PNG, GRAPH_DPI)
        }
    }

    @Test
    fun testSimpleTriangle() {
        saveMotionProfileGraph(
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
        saveMotionProfileGraph(
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
        saveMotionProfileGraph(
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
        saveMotionProfileGraph(
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
        saveMotionProfileGraph(
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
        saveMotionProfileGraph(
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
        saveMotionProfileGraph(
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
        saveMotionProfileGraph(
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