package com.acmerobotics.library

import kotlin.math.hypot

class Vector2d(val x: Double, val y: Double) {
    fun norm() = hypot(x, y)
    operator fun plus(other: Vector2d) = Vector2d(x + other.x, y + other.y)
    operator fun minus(other: Vector2d) = Vector2d(x - other.x, y - other.y)
    operator fun times(scalar: Double) = Vector2d(scalar * x, scalar * y)
    override fun toString() = String.format("(%.3f, %.3f)", x, y)
}

operator fun Double.times(vector: Vector2d) = vector.times(this)