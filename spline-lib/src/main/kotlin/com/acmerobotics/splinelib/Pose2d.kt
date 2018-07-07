package com.acmerobotics.splinelib

class Pose2d(val x: Double, val y: Double, val heading: Double) {
    constructor(pos: Vector2d, heading: Double) : this(pos.x, pos.y, heading)

    fun x() = x

    fun y() = y

    fun heading() = heading

    fun pos() = Vector2d(x, y)

    operator fun plus(other: Pose2d) = Pose2d(x + other.x, y + other.y, heading + other.heading)

    operator fun minus(other: Pose2d) = Pose2d(x - other.x, y - other.y, heading - other.heading)

    operator fun times(scalar: Double) = Pose2d(scalar * x, scalar * y, scalar * heading)

    operator fun div(scalar: Double) = Pose2d(x / scalar, y / scalar, heading / scalar)

    override fun toString() = String.format("(%.3f, %.3f, %.3f°)", x, y, Math.toDegrees(heading))
}

operator fun Double.times(pose: Pose2d) = pose.times(this)

operator fun Double.div(pose: Pose2d) = pose.div(this)