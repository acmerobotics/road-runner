package com.acmerobotics.library.util

import com.acmerobotics.library.Vector2d
import kotlin.math.abs

object MathUtil {
    fun curvature(deriv: Vector2d, secondDeriv: Vector2d): Double {
        val norm = deriv.norm()
        return abs(deriv.x * secondDeriv.y - deriv.y * secondDeriv.x) / (norm * norm * norm)
    }
}