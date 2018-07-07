package com.acmerobotics.splinelib

import kotlin.math.abs

object TestUtil {
    fun compareDerivatives(x: List<Double>, dx: List<Double>, ds: Double, epsilon: Double): Boolean {
        val numDx = (0 until x.size - 2).map { (x[it+2] - x[it]) / (2 * ds) }
        for (i in 2 until x.size - 2) {
            if (abs(numDx[i - 1] - dx[i]) > epsilon) {
                return false
            }
        }
        return true
    }
}