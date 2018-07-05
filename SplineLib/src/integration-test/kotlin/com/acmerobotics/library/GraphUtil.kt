package com.acmerobotics.library

import com.acmerobotics.library.profile.MotionProfile
import com.acmerobotics.library.spline.QuinticSpline
import com.acmerobotics.library.trajectory.Trajectory
import org.knowm.xchart.BitmapEncoder
import org.knowm.xchart.QuickChart
import org.knowm.xchart.XYChart
import java.io.File

object GraphUtil {
    private const val GRAPH_DIR = "./graphs/"
    private const val GRAPH_DPI = 300

    private fun saveGraph(name: String, graph: XYChart) {
        File(GRAPH_DIR).mkdirs()

        BitmapEncoder.saveBitmapWithDPI(graph, "$GRAPH_DIR$name", BitmapEncoder.BitmapFormat.PNG, GRAPH_DPI)
    }

    fun saveMotionProfile(name: String, profile: MotionProfile, includeAcceleration: Boolean = true, resolution: Int = 1000) {
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

        saveGraph(name, QuickChart.getChart(
            name,
            "time (sec)",
            "",
            labels.toTypedArray(),
            timeData,
            data.toTypedArray()
        ))
    }

    fun saveSpline(name: String, spline: QuinticSpline, resolution: Int = 1000) {
        val displacementData = (0..resolution).map { it / resolution.toDouble() * spline.length() }
        val points = displacementData.map { spline[it] }
        val xData = points.map { it.x }.toDoubleArray()
        val yData = points.map { it.y }.toDoubleArray()

        val graph = QuickChart.getChart(name, "x", "y", "spline", xData, yData)
        graph.styler.isLegendVisible = false
        saveGraph(name, graph)
    }

    fun saveSplineDerivatives(name: String, spline: QuinticSpline, resolution: Int = 1000) {
        val displacements = (0.. resolution).map { it / resolution.toDouble() * spline.length() }
        val pos = displacements.map { spline[it] }
        val derivs = displacements.map { spline.deriv(it) }
        val secondDerivs = displacements.map { spline.secondDeriv(it) }

        val posGraph = QuickChart.getChart("${name}Pos", "s", "", arrayOf("x", "y", "θ"), displacements.toDoubleArray(),
            arrayOf(pos.map { it.x }.toDoubleArray(), pos.map { it.y }.toDoubleArray(), pos.map { it.heading }.toDoubleArray())
        )
        val derivGraph = QuickChart.getChart("${name}Deriv", "s", "d/ds", arrayOf("dx/ds", "dy/ds", "dθ/ds"), displacements.toDoubleArray(),
            arrayOf(derivs.map { it.x }.toDoubleArray(), derivs.map { it.y }.toDoubleArray(), derivs.map { it.heading }.toDoubleArray())
        )
        val secondDerivGraph = QuickChart.getChart("${name}SecondDeriv", "s", "d2/ds2", arrayOf("d2x/ds2", "d2y/ds2", "d2θ/ds2"), displacements.toDoubleArray(),
            arrayOf(secondDerivs.map { it.x }.toDoubleArray(), secondDerivs.map { it.y }.toDoubleArray(), secondDerivs.map { it.heading }.toDoubleArray())
        )

        saveGraph("${name}Pos", posGraph)
        saveGraph("${name}Deriv", derivGraph)
        saveGraph("${name}SecondDeriv", secondDerivGraph)
    }

    fun saveTrajectory(name: String, trajectory: Trajectory, resolution: Int = 1000) {
        val timeData = (0..resolution).map { it / resolution.toDouble() * trajectory.duration() }.toDoubleArray()
        val velocityData = timeData.map { trajectory.velocity(it) }
        val xVelocityData = velocityData.map { it.x }.toDoubleArray()
        val yVelocityData = velocityData.map { it.y }.toDoubleArray()
        val omegaData = velocityData.map { it.heading }.toDoubleArray()

        val labels = listOf("vx(t)", "vy(t)", "ω(t)")
        val data = listOf(xVelocityData, yVelocityData, omegaData)

        saveGraph("${name}Trajectory", QuickChart.getChart(
            name,
            "time (sec)",
            "",
            labels.toTypedArray(),
            timeData,
            data.toTypedArray()
        ))
    }
}