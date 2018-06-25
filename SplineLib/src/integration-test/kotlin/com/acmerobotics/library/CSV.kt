package com.acmerobotics.library

import java.io.File

object CSV {
    private const val CSV_DIR = "./csv/"

    fun saveHolonomicPath(name: String, path: HolonomicPath, resolution: Int = 1000) {
        File(CSV_DIR).mkdirs()

        File("$CSV_DIR$name.csv").printWriter().use { out ->
            out.println("t,x,y,heading,dx,dy,omega,d2x,d2y,alpha")
            (0..resolution)
                .map { it / resolution.toDouble() * path.length() }
                .forEach {
                    val pos = path[it]
                    val deriv = path.deriv(it)
                    val secondDeriv = path.secondDeriv(it)
                    out.println("$it,${pos.x},${pos.y},${pos.heading},${deriv.x},${deriv.y},${deriv.heading},${secondDeriv.x},${secondDeriv.y},${secondDeriv.heading}")
                }
        }
    }
}