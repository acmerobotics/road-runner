package com.acmerobotics.library

import com.acmerobotics.library.spline.QuinticSpline
import java.io.File

object CSVUtil {
    private const val CSV_DIR = "./csv/"

    fun saveSpline(name: String, spline: QuinticSpline, resolution: Int = 1000) {
        File(CSV_DIR).mkdirs()

        File("$CSV_DIR$name.csv").printWriter().use { out ->
            out.println("t,x,y,heading,dx,dy,omega,d2x,d2y,alpha")
            val dx = spline.length() / resolution
            (0..resolution)
                .map { it * dx }
                .forEach {
                    val pos = spline[it]
                    val deriv = spline.deriv(it)
                    val secondDeriv = spline.secondDeriv(it)
                    out.println("$it,${pos.x},${pos.y},${pos.heading},${deriv.x},${deriv.y},${deriv.heading},${secondDeriv.x},${secondDeriv.y},${secondDeriv.heading}")
                }
        }
    }
}