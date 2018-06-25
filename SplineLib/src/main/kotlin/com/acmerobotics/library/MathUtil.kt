package com.acmerobotics.library

import kotlin.math.abs

object MathUtil {
    fun curvature(deriv: Vector2d, secondDeriv: Vector2d): Double {
        val norm = deriv.norm()
        return abs(deriv.x * secondDeriv.y - deriv.y * secondDeriv.x) / (norm * norm * norm)
    }
}