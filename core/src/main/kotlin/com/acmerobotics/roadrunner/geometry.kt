import com.acmerobotics.roadrunner.geometry.Vector2d

import kotlin.math.*

class Twist2d(
    val x: Double,
    val y: Double,
    val heading: Double,
)

class Vector2d(
    val x: Double,
    val y: Double,
) {
    fun rotated(angle: Double) = Vector2d(
        x * cos(angle) - y * sin(angle),
        x * sin(angle) + y * cos(angle),
    )
}

class Pose2d(
    val x: Double,
    val y: Double,
    val heading: Double,
) {
    constructor(pos: Vector2d, heading: Double) : this(pos.x, pos.y, heading)

    fun vec() = Vector2d(x, y)

    // pose error replacement
//    operator fun minus()

    // relative odometry update replacement
//    operator fun plus()

    // field transform replacement
//    operator fun times()
}
