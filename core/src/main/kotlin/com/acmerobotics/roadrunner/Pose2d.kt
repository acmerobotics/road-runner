package com.acmerobotics.roadrunner

/**
 * Class for representing 2D robot poses (x, y, and heading) and their derivatives.
 */
class Pose2d @JvmOverloads constructor(
        val x: Double = 0.0,
        val y: Double = 0.0,
        val heading: Double = 0.0
) {
    constructor(pos: Vector2d, heading: Double) : this(pos.x, pos.y, heading)

    fun pos() = Vector2d(x, y)

    operator fun plus(other: Pose2d) = Pose2d(x + other.x, y + other.y, heading + other.heading)

    operator fun minus(other: Pose2d) = Pose2d(x - other.x, y - other.y, heading - other.heading)

    operator fun times(scalar: Double) = Pose2d(scalar * x, scalar * y, scalar * heading)

    operator fun div(scalar: Double) = Pose2d(x / scalar, y / scalar, heading / scalar)

    operator fun unaryMinus() = Pose2d(-x, -y, -heading)

    override fun toString() = String.format("(%.3f, %.3f, %.3f°)", x, y, Math.toDegrees(heading))
}

operator fun Double.times(pose: Pose2d) = pose.times(this)

operator fun Double.div(pose: Pose2d) = pose.div(this)