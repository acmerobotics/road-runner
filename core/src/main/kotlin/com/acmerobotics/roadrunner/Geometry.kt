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
    }

    operator fun plus(x: Double) = this * exp(x)
    operator fun minus(r: Rotation2d) = (r.inverse() * this).log()

    operator fun times(v: Vector2d) = Vector2d(real * v.x - imag * v.y, imag * v.x + real * v.y)
    operator fun times(t: Twist2d) = Twist2d(this * t.transVel, t.rotVel)
    operator fun times(r: Rotation2d) =
        Rotation2d(real * r.real - imag * r.imag, real * r.imag + imag * r.real)

    fun vec() = Vector2d(real, imag)

    fun inverse() = Rotation2d(real, -imag)

    /**
     * Get the rotation as a normalized [Double].
     */
    fun log() = atan2(imag, real)
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

    operator fun times(t: Twist2dDual<Param>) = Twist2dDual(this * t.transVel, t.rotVel)
    operator fun times(v: Vector2dDual<Param>) = Vector2dDual(real * v.x - imag * v.y, imag * v.x + real * v.y)
    operator fun times(v: Vector2d) = Vector2dDual(real * v.x - imag * v.y, imag * v.x + real * v.y)
    operator fun times(r: Rotation2dDual<Param>) =
        Rotation2dDual(real * r.real - imag * r.imag, real * r.imag + imag * r.real)
    operator fun times(r: Rotation2d) =
        Rotation2dDual(real * r.real - imag * r.imag, real * r.imag + imag * r.real)
    operator fun times(t: Pose2d) = Pose2dDual(this * t.trans, this * t.rot)

    fun inverse() = Rotation2dDual(real, -imag)

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
        Rotation2dDual(real.reparam(oldParam), imag.reparam(oldParam))

    // derivative of atan2 under unit norm assumption
    fun velocity() = real * imag.drop(1) - imag * real.drop(1)
    fun value() = Rotation2d(real.value(), imag.value())

    fun withRot(r: Rotation2d) = Rotation2dDual(real.withFirst(r.real), imag.withFirst(r.imag))
}

/**
 * @usesMathJax
 *
 * 2D rigid transform comprised of [rot] followed by [trans].
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
    val trans: Vector2d,
    @JvmField
    val rot: Rotation2d,
) {
    constructor(trans: Vector2d, rot: Double) : this(trans, Rotation2d.exp(rot))
    constructor(transX: Double, transY: Double, rot: Double) : this(Vector2d(transX, transY), rot)

    companion object {
        @JvmStatic
        fun exp(incr: Twist2dIncr): Pose2d {
            val rotation = Rotation2d.exp(incr.rotIncr)

            val u = incr.rotIncr + snz(incr.rotIncr)
            val c = 1 - cos(u)
            val s = sin(u)
            val translation = Vector2d(
                (s * incr.transIncr.x - c * incr.transIncr.y) / u,
                (c * incr.transIncr.x + s * incr.transIncr.y) / u
            )

            return Pose2d(translation, rotation)
        }
    }

    operator fun plus(t: Twist2dIncr) = this * exp(t)
    fun minusExp(t: Pose2d) = t.inverse() * this
    operator fun minus(t: Pose2d) = minusExp(t).log()

    operator fun times(t: Pose2d) = Pose2d(rot * t.trans + trans, rot * t.rot)
    operator fun times(v: Vector2d) = rot * v + trans
    operator fun times(t: Twist2d) = Twist2d(rot * t.transVel, t.rotVel)

    fun inverse() = Pose2d(rot.inverse() * -trans, rot.inverse())

    fun log(): Twist2dIncr {
        val theta = rot.log()

        val halfu = 0.5 * theta + snz(theta)
        val v = halfu / tan(halfu)
        return Twist2dIncr(
            Vector2d(
                v * trans.x + halfu * trans.y,
                -halfu * trans.x + v * trans.y,
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
    val trans: Vector2dDual<Param>,
    @JvmField
    val rot: Rotation2dDual<Param>,
) {
    companion object {
        @JvmStatic
        fun <Param> constant(t: Pose2d, n: Int) =
            Pose2dDual<Param>(Vector2dDual.constant(t.trans, n), Rotation2dDual.constant(t.rot, n))
    }

    operator fun plus(t: Twist2dIncr) = this * Pose2d.exp(t)

    operator fun times(t: Pose2d) = Pose2dDual(rot * t.trans + trans, rot * t.rot)
    operator fun times(t: Pose2dDual<Param>) = Pose2dDual(rot * t.trans + trans, rot * t.rot)
    operator fun times(t: Twist2dDual<Param>) = Twist2dDual(rot * t.transVel, t.rotVel)

    fun inverse() = rot.inverse().let {
        Pose2dDual(it * -trans, it)
    }

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
        Pose2dDual(trans.reparam(oldParam), rot.reparam(oldParam))

    fun value() = Pose2d(trans.value(), rot.value())
    fun velocity() = Twist2dDual(trans.drop(1), rot.velocity())
}

/**
 * 2D twist (velocities)
 */
data class Twist2d(@JvmField val transVel: Vector2d, @JvmField val rotVel: Double) {
    operator fun minus(t: Twist2d) = Twist2d(transVel - t.transVel, rotVel - t.rotVel)
}

/**
 * Dual version of [Twist2d].
 */
data class Twist2dDual<Param>(@JvmField val transVel: Vector2dDual<Param>, @JvmField val rotVel: DualNum<Param>) {
    companion object {
        @JvmStatic
        fun <Param> constant(t: Twist2d, n: Int) =
            Twist2dDual<Param>(Vector2dDual.constant(t.transVel, n), DualNum.constant(t.rotVel, n))
    }

    operator fun plus(other: Twist2d) = Twist2dDual(transVel + other.transVel, rotVel + other.rotVel)

    fun value() = Twist2d(transVel.value(), rotVel.value())
}

data class Twist2dIncr(@JvmField val transIncr: Vector2d, @JvmField val rotIncr: Double)

data class Twist2dIncrDual<Param>(
    @JvmField val transIncr: Vector2dDual<Param>,
    @JvmField val rotIncr: DualNum<Param>
) {
    fun value() = Twist2dIncr(transIncr.value(), rotIncr.value())
    fun velocity() = Twist2dDual(transIncr.drop(1), rotIncr.drop(1))
}
