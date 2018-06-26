package com.acmerobotics.library

import kotlin.math.sqrt

class Vector2d(val x: Double, val y: Double) {
    fun norm() = sqrt(x*x + y*y)
    operator fun plus(other: Vector2d) = Vector2d(x + other.x, y + other.y)
    operator fun minus(other: Vector2d) = Vector2d(x - other.x, y - other.y)
    operator fun times(scalar: Double) = Vector2d(scalar * x, scalar * y)
    operator fun div(scalar: Double) = Vector2d(x / scalar, y / scalar)
    operator fun Double.times(vector: Vector2d) = vector.times(this)
    operator fun Double.div(vector: Vector2d) = vector.div(this)
    override fun toString() = String.format("(%.3f, %.3f)", x, y)
}