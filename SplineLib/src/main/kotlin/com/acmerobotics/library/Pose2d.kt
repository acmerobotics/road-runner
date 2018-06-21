package com.acmerobotics.library

class Pose2d(val x: Double, val y: Double, val heading: Double) {
    fun pos() = Vector2d(x, y)
    override fun toString() = String.format("(%.3f, %.3f, %.3f)", x, y, Math.toDegrees(heading))
}