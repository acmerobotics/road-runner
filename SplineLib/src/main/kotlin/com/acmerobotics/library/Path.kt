package com.acmerobotics.library

import kotlin.math.abs

abstract class Path {
    abstract fun length(): Double
    abstract operator fun get(displacement: Double): Vector2d
    abstract fun deriv(displacement: Double): Vector2d
    abstract fun secondDeriv(displacement: Double): Vector2d
    abstract fun thirdDeriv(displacement: Double): Vector2d

    fun start() = get(0.0)
    fun end() = get(length())

    fun curvature(displacement: Double): Double {
        val deriv = deriv(displacement)
        val secondDeriv = secondDeriv(displacement)
        val norm = deriv.norm()
        return abs(deriv.x * secondDeriv.y - deriv.y * secondDeriv.x) / (norm * norm * norm)
    }
}