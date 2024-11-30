@file:JvmName("Geometry")

package com.acmerobotics.roadrunner

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * @usesMathJax
 *
 * Vector \((x, y)\)
 */
data class Vector2d(@JvmField val x: Double, @JvmField val y: Double) {
    operator fun plus(v: Vector2d) = Vector2d(x + v.x, y + v.y)
    operator fun minus(v: Vector2d) = Vector2d(x - v.x, y - v.y)
    operator fun unaryMinus() = Vector2d(-x, -y)

    operator fun times(z: Double) = Vector2d(x * z, y * z)
    operator fun div(z: Double) = Vector2d(x / z, y / z)

    infix fun dot(v: Vector2d) = x * v.x + y * v.y
    fun sqrNorm() = this dot this
    fun norm() = sqrt(sqrNorm())

    // precondition: this is normalized
    fun angleCast() = Rotation2d(x, y)
}

/**
 * Dual version of [Vector2d].
 */
data class Vector2dDual<Param>(@JvmField val x: DualNum<Param>, @JvmField val y: DualNum<Param>) {
    companion object {
        @JvmStatic
        fun <Param> constant(v: Vector2d, n: Int) =
            Vector2dDual<Param>(DualNum.constant(v.x, n), DualNum.constant(v.y, n))
    }

    operator fun plus(v: Vector2d) = Vector2dDual(x + v.x, y + v.y)
    operator fun plus(v: Vector2dDual<Param>) = Vector2dDual(x + v.x, y + v.y)
    operator fun minus(v: Vector2dDual<Param>) = Vector2dDual(x - v.x, y - v.y)
    operator fun unaryMinus() = Vector2dDual(-x, -y)

    operator fun div(z: Double) = Vector2dDual(x / z, y / z)

    infix fun dot(v: Vector2dDual<Param>) = x * v.x + y * v.y
    fun sqrNorm() = this dot this
    fun norm() = sqrNorm().sqrt()

    fun bind() = Vector2dDual(x, y)

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
        Vector2dDual(x.reparam(oldParam), y.reparam(oldParam))

    fun drop(n: Int) = Vector2dDual(x.drop(n), y.drop(n))
    fun value() = Vector2d(x.value(), y.value())

    // precondition: this is normalized
    fun angleCast() = Rotation2dDual(x, y)
}

/**
 * @usesMathJax
 *
 * Rotation \(\theta\) represented by the unit circle point \((\cos \theta, \sin \theta)\) or unit-modulus complex
 * number \(\cos \theta + i \sin \theta\).
 *
 * Advanced: Rotations in two dimensions comprise a Lie group referred to as SO(2). The terminology [exp] and [log]
 * comes from the Lie theory, and [this paper](https://arxiv.org/abs/1812.01537) gives a targeted exposition of the key
 * fundamentals.
 */
data class Rotation2d(@JvmField val real: Double, @JvmField val imag: Double) {
    companion object {
        /**
         * Turns an unnormalized angle into a rotation.
         */
        @JvmStatic
        fun exp(theta: Double) = Rotation2d(cos(theta), sin(theta))

        /**
         * Alias for [exp].
         */
        @JvmStatic
        fun fromDouble(theta: Double) = exp(theta)
    }

    operator fun plus(x: Double) = this * exp(x)
    operator fun minus(r: Rotation2d) = (r.inverse() * this).log()

    operator fun times(v: Vector2d) = Vector2d(real * v.x - imag * v.y, imag * v.x + real * v.y)
    operator fun times(pv: PoseVelocity2d) = PoseVelocity2d(this * pv.linearVel, pv.angVel)
    operator fun times(r: Rotation2d) =
        Rotation2d(real * r.real - imag * r.imag, real * r.imag + imag * r.real)

    fun vec() = Vector2d(real, imag)

    fun inverse() = Rotation2d(real, -imag)

    /**
     * Get the rotation as a normalized [Double].
     */
    fun log() = atan2(imag, real)

    /**
     * Alias for [log].
     */
    fun toDouble() = log()
}

/**
 * Dual version of [Rotation2d].
 */
data class Rotation2dDual<Param>(@JvmField val real: DualNum<Param>, @JvmField val imag: DualNum<Param>) {
    init {
        require(real.size() == imag.size())
        require(real.size() <= 3)
    }

    companion object {
        @JvmStatic
        fun <Param> exp(theta: DualNum<Param>) = Rotation2dDual(theta.cos(), theta.sin())

        @JvmStatic
        fun <Param> constant(r: Rotation2d, n: Int) =
            Rotation2dDual<Param>(DualNum.constant(r.real, n), DualNum.constant(r.imag, n))
    }

    fun size() = real.size()

    operator fun plus(x: Double) = this * Rotation2d.exp(x)
    operator fun plus(d: DualNum<Param>) = this * exp(d)

    operator fun times(pv: PoseVelocity2dDual<Param>) = PoseVelocity2dDual(this * pv.linearVel, pv.angVel)
    operator fun times(v: Vector2dDual<Param>) = Vector2dDual(real * v.x - imag * v.y, imag * v.x + real * v.y)
    operator fun times(v: Vector2d) = Vector2dDual(real * v.x - imag * v.y, imag * v.x + real * v.y)
    operator fun times(r: Rotation2dDual<Param>) =
        Rotation2dDual(real * r.real - imag * r.imag, real * r.imag + imag * r.real)
    operator fun times(r: Rotation2d) =
        Rotation2dDual(real * r.real - imag * r.imag, real * r.imag + imag * r.real)
    operator fun times(p: Pose2d) = Pose2dDual(this * p.position, this * p.heading)

    fun inverse() = Rotation2dDual(real, -imag)

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
        Rotation2dDual(real.reparam(oldParam), imag.reparam(oldParam))

    // derivative of atan2 under unit norm assumption
    fun velocity() = real * imag.drop(1) - imag * real.drop(1)
    fun value() = Rotation2d(real.value(), imag.value())
}

/**
 * @usesMathJax
 *
 * 2D rigid transform comprised of [heading] followed by [position].
 *
 * The pose `destPoseSource` denotes the transform from frame `Source` into frame `Dest`. It can be applied with
 * `times()` to change the coordinates of `xSource` into `xDest` where `x` is a vector, twist, or even another pose:
 * `xDest = destPoseSource * xSource`. The awkward names take some getting used to, but they avoid many routine errors.
 *
 * Transforms into the world frame are common enough to warrant a shorthand. The pose `worldPoseSource` can be shortened
 * to `poseSource` for any frame `Source`.
 *
 * Advanced: Transforms in two dimensions comprise a Lie group referred to as SE(2). The terminology [exp] and [log]
 * comes from the Lie theory, and [this paper](https://arxiv.org/abs/1812.01537) gives a targeted exposition of the key
 * fundamentals.
 */
data class Pose2d(
    @JvmField
    val position: Vector2d,
    @JvmField
    val heading: Rotation2d,
) {
    constructor(position: Vector2d, heading: Double) : this(position, Rotation2d.exp(heading))
    constructor(positionX: Double, positionY: Double, heading: Double) : this(Vector2d(positionX, positionY), heading)

    companion object {
        @JvmStatic
        fun exp(t: Twist2d): Pose2d {
            val heading = Rotation2d.exp(t.angle)

            val u = t.angle + snz(t.angle)
            val c = 1 - cos(u)
            val s = sin(u)
            val translation = Vector2d(
                (s * t.line.x - c * t.line.y) / u,
                (c * t.line.x + s * t.line.y) / u
            )

            return Pose2d(translation, heading)
        }
    }

    operator fun plus(t: Twist2d) = this * exp(t)
    fun minusExp(t: Pose2d) = t.inverse() * this
    operator fun minus(t: Pose2d) = minusExp(t).log()

    operator fun times(p: Pose2d) = Pose2d(heading * p.position + position, heading * p.heading)
    operator fun times(v: Vector2d) = heading * v + position
    operator fun times(pv: PoseVelocity2d) = PoseVelocity2d(heading * pv.linearVel, pv.angVel)

    fun inverse() = Pose2d(heading.inverse() * -position, heading.inverse())

    fun log(): Twist2d {
        val theta = heading.log()

        val halfu = 0.5 * theta + snz(theta)
        val v = halfu / tan(halfu)
        return Twist2d(
            Vector2d(
                v * position.x + halfu * position.y,
                -halfu * position.x + v * position.y,
            ),
            theta,
        )
    }
}

/**
 * Dual version of [Pose2d].
 */
data class Pose2dDual<Param>(
    @JvmField
    val position: Vector2dDual<Param>,
    @JvmField
    val heading: Rotation2dDual<Param>,
) {
    constructor(positionX: DualNum<Param>, positionY: DualNum<Param>, heading: Rotation2dDual<Param>) :
        this(Vector2dDual(positionX, positionY), heading)
    constructor(positionX: DualNum<Param>, positionY: DualNum<Param>, heading: DualNum<Param>) :
        this(positionX, positionY, Rotation2dDual.exp(heading))

    companion object {
        @JvmStatic
        fun <Param> constant(p: Pose2d, n: Int) =
            Pose2dDual<Param>(Vector2dDual.constant(p.position, n), Rotation2dDual.constant(p.heading, n))
    }

    operator fun plus(t: Twist2d) = this * Pose2d.exp(t)

    operator fun times(p: Pose2d) = Pose2dDual(heading * p.position + position, heading * p.heading)
    operator fun times(p: Pose2dDual<Param>) = Pose2dDual(heading * p.position + position, heading * p.heading)
    operator fun times(pv: PoseVelocity2dDual<Param>) = PoseVelocity2dDual(heading * pv.linearVel, pv.angVel)

    fun inverse() = heading.inverse().let {
        Pose2dDual(it * -position, it)
    }

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
        Pose2dDual(position.reparam(oldParam), heading.reparam(oldParam))

    fun value() = Pose2d(position.value(), heading.value())
    fun velocity() = PoseVelocity2dDual(position.drop(1), heading.velocity())
}

data class PoseVelocity2d(@JvmField val linearVel: Vector2d, @JvmField val angVel: Double) {
    operator fun minus(pv: PoseVelocity2d) = PoseVelocity2d(linearVel - pv.linearVel, angVel - pv.angVel)
}

/**
 * Dual version of [PoseVelocity2d].
 */
data class PoseVelocity2dDual<Param>(
    @JvmField val linearVel: Vector2dDual<Param>,
    @JvmField val angVel: DualNum<Param>
) {
    companion object {
        @JvmStatic
        fun <Param> constant(pv: PoseVelocity2d, n: Int) =
            PoseVelocity2dDual<Param>(Vector2dDual.constant(pv.linearVel, n), DualNum.constant(pv.angVel, n))
    }

    operator fun plus(other: PoseVelocity2d) = PoseVelocity2dDual(linearVel + other.linearVel, angVel + other.angVel)

    fun value() = PoseVelocity2d(linearVel.value(), angVel.value())
}

data class Twist2d(@JvmField val line: Vector2d, @JvmField val angle: Double)

data class Twist2dDual<Param>(
    @JvmField val line: Vector2dDual<Param>,
    @JvmField val angle: DualNum<Param>
) {
    fun value() = Twist2d(line.value(), angle.value())
    fun velocity() = PoseVelocity2dDual(line.drop(1), angle.drop(1))
}
