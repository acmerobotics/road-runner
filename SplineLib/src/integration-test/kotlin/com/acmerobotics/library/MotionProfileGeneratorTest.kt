package com.acmerobotics.library

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import org.knowm.xchart.BitmapEncoder
import org.knowm.xchart.BitmapEncoder.BitmapFormat
import org.knowm.xchart.QuickChart
import java.io.File


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class MotionProfileGeneratorTest {
    companion object {
        private const val GRAPH_DIR = "./graphs/"
        private const val GRAPH_DPI = 300

        fun saveMotionProfileGraph(name: String, profile: MotionProfile, resolution: Int = 1000) {
            val timeData = (0..resolution).map { it / resolution.toDouble() * profile.duration() }.toDoubleArray()
            val positionData = timeData.map { profile[it].x }.toDoubleArray()
            val velocityData = timeData.map { profile[it].v }.toDoubleArray()
            val accelerationData = timeData.map { profile[it].a }.toDoubleArray()

            val graph = QuickChart.getChart(
                name,
                "time (s)",
                "",
                arrayOf("x(t)", "v(t)", "a(t)"),
                timeData,
                arrayOf(positionData, velocityData, accelerationData)
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
}