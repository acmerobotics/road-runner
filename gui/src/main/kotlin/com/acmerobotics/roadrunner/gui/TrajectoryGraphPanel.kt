package com.acmerobotics.roadrunner.gui

import com.acmerobotics.roadrunner.trajectory.Trajectory
import java.awt.Color
import java.awt.Graphics
import java.awt.Graphics2D
import javax.swing.JPanel
import org.knowm.xchart.QuickChart
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.MatlabTheme
import org.knowm.xchart.style.Styler

private const val RESOLUTION = 1000
val SERIES_COLORS = arrayOf(
        Color(0x2979ff), Color(0xdd2c00), Color(0x4caf50)
)

class TrajectoryGraphPanel : JPanel() {

    private var chart: XYChart = QuickChart.getChart("", "time", "velocity",
            arrayOf("ẋ ", "ẏ ", "ω"), doubleArrayOf(0.0), arrayOf(doubleArrayOf(0.0), doubleArrayOf(0.0), doubleArrayOf(0.0)))

    override fun paintComponent(g: Graphics?) {
        super.paintComponent(g)

        val g2d = g as Graphics2D
        chart.paint(g2d, width, height)
        g2d.dispose()
    }

    fun updateTrajectory(trajectory: Trajectory) {
        if (trajectory.duration().isInfinite() || trajectory.duration().isNaN()) {
            return
        }

        val dt = trajectory.duration() / RESOLUTION
        val t = (0..RESOLUTION).map { it * dt }.toDoubleArray()
        val velocity = t.map { trajectory.velocity(it) }
        val x = velocity.map { it.x }.toDoubleArray()
        val y = velocity.map { it.y }.toDoubleArray()
        val heading = velocity.map { it.heading.toDegrees() }.toDoubleArray()

        chart = QuickChart.getChart("", "", "", arrayOf("ẋ ", "ẏ ", "ω"), t, arrayOf(x, y, heading))
        chart.styler.theme = MatlabTheme()
        chart.styler.legendPosition = Styler.LegendPosition.InsideNE
        chart.styler.chartBackgroundColor = background
        chart.styler.plotBackgroundColor = background
        chart.styler.legendBackgroundColor = background
        chart.styler.legendBorderColor = foreground
        chart.styler.axisTickLabelsColor = foreground
        chart.styler.chartFontColor = foreground
        chart.styler.plotBorderColor = foreground
        chart.styler.plotGridLinesColor = foreground
        chart.styler.seriesColors = SERIES_COLORS

        repaint()
    }
}
