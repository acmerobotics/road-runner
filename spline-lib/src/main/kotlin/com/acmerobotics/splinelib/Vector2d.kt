package com.acmerobotics.splinelib

import kotlin.math.sqrt

class Vector2d(val x: Double, val y: Double) {
    fun x() = x

    fun y() = y

    fun norm() = sqrt(x*x + y*y)

    operator fun plus(other: Vector2d) = Vector2d(x + other.x, y + other.y)

    operator fun minus(other: Vector2d) = Vector2d(x - other.x, y - other.y)

    operator fun times(scalar: Double) = Vector2d(scalar * x, scalar * y)

    operator fun div(scalar: Double) = Vector2d(x / scalar, y / scalar)

    infix fun distanceTo(other: Vector2d) = (this - other).norm()

    fun rotated(angle: Double): Vector2d {
        val newX = x * Math.cos(angle) - y * Math.sin(angle)
        val newY = x * Math.sin(angle) + y * Math.cos(angle)
        return Vector2d(newX, newY)
    }

    override fun toString() = String.format("(%.3f, %.3f)", x, y)
}

operator fun Double.times(vector: Vector2d) = vector.times(this)

operator fun Double.div(vector: Vector2d) = vector.div(this)