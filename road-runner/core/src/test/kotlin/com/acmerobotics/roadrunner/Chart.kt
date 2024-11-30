package com.acmerobotics.roadrunner

import org.knowm.xchart.BitmapEncoder
import org.knowm.xchart.XYChart
import java.awt.image.BufferedImage
import java.io.File
import java.nio.file.Paths
import javax.imageio.ImageIO

const val CHART_DIR = "./charts/"
const val CHART_DPI = 300

fun saveChart(name: String, chart: XYChart) {
    val file = File(Paths.get(CHART_DIR, name).toString())
    file.parentFile.mkdirs()

    BitmapEncoder.saveBitmapWithDPI(chart, file.absolutePath, BitmapEncoder.BitmapFormat.PNG, CHART_DPI)
}

fun saveChartPanel(name: String, size: Int, charts: List<XYChart>) {
    val file = File(Paths.get(CHART_DIR, "$name.png").toString())
    file.parentFile.mkdirs()

    val im = BufferedImage(size * charts.size, size, BufferedImage.TYPE_INT_RGB)
    val g = im.createGraphics()
    for (chart in charts) {
        chart.paint(g, size, size)
        g.translate(size, 0)
    }
    ImageIO.write(im, "png", file)
}
