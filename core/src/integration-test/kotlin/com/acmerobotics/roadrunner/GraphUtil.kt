package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.path.ParametricCurve
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.util.DoubleProgression
import java.io.File
import java.nio.file.Paths
import org.knowm.xchart.BitmapEncoder
import org.knowm.xchart.QuickChart
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.MatlabTheme

const val GRAPH_DIR = "./graphs/"
const val GRAPH_DPI = 300

/**
 * Useful utilities for generating and saving graphs.
 */
object GraphUtil {

    fun saveGraph(name: String, graph: XYChart) {
        val file = File(Paths.get(GRAPH_DIR, name).toString())
        file.parentFile.mkdirs()

        BitmapEncoder.saveBitmapWithDPI(graph, file.absolutePath, BitmapEncoder.BitmapFormat.PNG, GRAPH_DPI)
    }

    fun saveMotionProfile(
        name: String,
        profile: MotionProfile,
        includeAcceleration: Boolean = true,
        resolution: Int = 1000
    ) {
        val timeData = DoubleProgression.fromClosedInterval(
            0.0, profile.duration(), resolution).toList().toDoubleArray()
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
        graph.styler.theme = MatlabTheme()

        saveGraph(name, graph)
    }

    fun saveParametricCurve(name: String, parametricCurve: ParametricCurve, resolution: Int = 1000) {
        val displacementData = DoubleProgression.fromClosedInterval(
            0.0, parametricCurve.length(), resolution)
        val points = displacementData.map { parametricCurve[it] }
        val xData = points.map { it.x }.toDoubleArray()
        val yData = points.map { it.y }.toDoubleArray()

        val graph = QuickChart.getChart(name, "x", "y", name, xData, yData)
        graph.styler.isLegendVisible = false
        graph.styler.theme = MatlabTheme()
        saveGraph(name, graph)
    }

    fun savePath(name: String, path: Path, resolution: Int = 1000) {
        val displacementData = DoubleProgression.fromClosedInterval(
            0.0, path.length(), resolution).toList().toDoubleArray()
        val posData = displacementData.map { path[it] }
        val xData = posData.map { it.x }.toDoubleArray()
        val yData = posData.map { it.y }.toDoubleArray()

        val graph = QuickChart.getChart(
                name,
                "",
                "",
                "path",
                xData,
                yData
        )
        graph.styler.theme = MatlabTheme()

        saveGraph(name, graph)
    }

    fun savePathPositions(name: String, path: Path, resolution: Int = 1000) {
        val displacementData = DoubleProgression.fromClosedInterval(
            0.0, path.length(), resolution).toList().toDoubleArray()
        val posData = displacementData.map { path[it] }
        val xData = posData.map { it.x }.toDoubleArray()
        val yData = posData.map { it.y }.toDoubleArray()
        val headingData = posData.map { Math.toDegrees(it.heading) }.toDoubleArray()

        val labels = listOf("x(s)", "y(s)", "θ(s)")
        val data = listOf(xData, yData, headingData)

        val graph = QuickChart.getChart(
                name,
                "displacement",
                "",
                labels.toTypedArray(),
                displacementData,
                data.toTypedArray()
        )
        graph.styler.theme = MatlabTheme()

        saveGraph(name, graph)
    }

    fun savePathDerivatives(name: String, path: Path, resolution: Int = 1000) {
        val displacementData = DoubleProgression.fromClosedInterval(
            0.0, path.length(), resolution).toList().toDoubleArray()
        val derivData = displacementData.map { path.deriv(it) }
        val xDerivData = derivData.map { it.x }.toDoubleArray()
        val yDerivData = derivData.map { it.y }.toDoubleArray()
        val thetaDerivData = derivData.map { Math.toDegrees(it.heading) }.toDoubleArray()

        val labels = listOf("dx/ds", "dy/ds", "dθ/ds")
        val data = listOf(xDerivData, yDerivData, thetaDerivData)

        val graph = QuickChart.getChart(
                name,
                "displacement",
                "",
                labels.toTypedArray(),
                displacementData,
                data.toTypedArray()
        )
        graph.styler.theme = MatlabTheme()

        saveGraph(name, graph)
    }

    fun saveTrajectory(name: String, trajectory: Trajectory, resolution: Int = 1000) {
        val timeData = DoubleProgression.fromClosedInterval(
            0.0, trajectory.duration(), resolution).toList().toDoubleArray()
        val velocityData = timeData.map { trajectory.velocity(it) }
        val xVelocityData = velocityData.map { it.x }.toDoubleArray()
        val yVelocityData = velocityData.map { it.y }.toDoubleArray()
        val omegaData = velocityData.map { it.heading }.toDoubleArray()

        val labels = listOf("dx/dt", "dy/dt", "ω")
        val data = listOf(xVelocityData, yVelocityData, omegaData)

        val graph = QuickChart.getChart(
                name,
                "time (sec)",
                "",
                labels.toTypedArray(),
                timeData,
                data.toTypedArray()
        )
        graph.styler.theme = MatlabTheme()

        saveGraph(name, graph)
    }
}
