package com.acmerobotics.library

import com.acmerobotics.library.path.Path
import java.io.File

object CSVUtil {
    private const val CSV_DIR = "./csv/"

    fun savePath(name: String, path: Path, resolution: Int = 1000) {
        File(CSV_DIR).mkdirs()

        File("$CSV_DIR$name.csv").printWriter().use { out ->
            out.println("t,x,y,heading,dx,dy,omega,d2x,d2y,alpha,num_d2x")
            val dx = path.length() / resolution
            (0..resolution)
                .map { it * dx }
                .forEach {
                    val pos = path[it]
                    val deriv = path.deriv(it)
                    val secondDeriv = path.secondDeriv(it)
                    out.println("$it,${pos.x},${pos.y},${pos.heading},${deriv.x},${deriv.y},${deriv.heading},${secondDeriv.x},${secondDeriv.y},${secondDeriv.heading},${(path.deriv(it+dx).x-path.deriv(it-dx).x)/(2.0*dx)}")
                }
        }
    }
}