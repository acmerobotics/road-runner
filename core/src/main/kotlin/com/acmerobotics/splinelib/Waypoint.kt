package com.acmerobotics.splinelib

class Waypoint @JvmOverloads constructor(
    val x: Double,
    val y: Double,
    var dx: Double = 0.0,
    var dy: Double = 0.0,
    var dx2: Double = 0.0,
    var dy2: Double = 0.0
) {
    fun pos() = Vector2d(x, y)

    fun deriv() = Vector2d(dx, dy)

    fun secondDeriv() = Vector2d(dx2, dy2)
}