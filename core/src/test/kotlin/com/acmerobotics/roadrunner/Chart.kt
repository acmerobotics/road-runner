package com.acmerobotics.roadrunner

import org.knowm.xchart.BitmapEncoder
import org.knowm.xchart.XYChart
import java.io.File
import java.nio.file.Paths

const val CHART_DIR = "./charts/"
const val CHART_DPI = 300

fun saveChart(name: String, chart: XYChart) {
    val file = File(Paths.get(CHART_DIR, name).toString())
    file.parentFile.mkdirs()

    BitmapEncoder.saveBitmapWithDPI(chart, file.absolutePath, BitmapEncoder.BitmapFormat.PNG, CHART_DPI)
}
