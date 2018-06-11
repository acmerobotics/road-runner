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
        private const val GRAPH_DIR = "./graph/"
        private const val GRAPH_DPI = 300

        fun saveMotionProfileGraph(name: String, profile: MotionProfile, resolution: Int = 1000) {
            val xData = (0..resolution).map { it / resolution.toDouble() * profile.duration() }
            val yData = xData.map { profile[it].v }

            val graph = QuickChart.getChart(name, "time (s)", "", "v(t)", xData, yData)

            File(GRAPH_DIR).mkdirs()

            BitmapEncoder.saveBitmapWithDPI(graph, "$GRAPH_DIR$name", BitmapFormat.PNG, GRAPH_DPI)
        }
    }

    @Test
    fun test() {
        val profile = MotionProfileGenerator.generateMotionProfile(
            MotionState(5.0, 0.0, 0.0),
            MotionState(25.0, 5.0, 0.0),
            { _ -> 10.0 },
            { _ -> 5.0 },
            4
        )
        saveMotionProfileGraph("test", profile)
    }
}