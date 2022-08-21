@file:JvmName("Geometry")

package com.acmerobotics.roadrunner

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * @usesMathJax
 *
 * Position \((x, y)\)
 */
data class Position2d(@JvmField val x: Double, @JvmField val y: Double) {
    operator fun plus(v: Vector2d) = Position2d(x + v.x, y + v.y)
    operator fun minus(p: Position2d) = Vector2d(x - p.x, y - p.y)
}

/**
 * Dual version of [Position2d].
 */
data class Position2dDual<Param>(@JvmField val x: DualNum<Param>, @JvmField val y: DualNum<Param>) {
    companion object {
        @JvmStatic
        fun <Param> constant(p: Position2d, n: Int) = Position2dDual<Param>(
            DualNum.constant(p.x, n), DualNum.constant(p.y, n)
        )
    }

    operator fun minus(p: Position2dDual<Param>) = Vector2dDual(x - p.x, y - p.y)

    fun free() = Vector2dDual(x, y)

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) = Position2dDual(x.reparam(oldParam), y.reparam(oldParam))

    fun value() = Position2d(x.value(), y.value())
    fun tangentVec() = Vector2dDual(x.drop(1), y.drop(1))
}

fun Position2dDual<Arclength>.tangent() = Rotation2dDual(x.drop(1), y.drop(1))

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

    fun bind() = Position2d(x, y)
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
    operator fun plus(p: Position2d) = Position2dDual(x + p.x, y + p.y)
    operator fun minus(v: Vector2dDual<Param>) = Vector2dDual(x - v.x, y - v.y)
    operator fun unaryMinus() = Vector2dDual(-x, -y)

    operator fun div(z: Double) = Vector2dDual(x / z, y / z)

    infix fun dot(v: Vector2dDual<Param>) = x * v.x + y * v.y
    fun sqrNorm() = this dot this
    fun norm() = sqrNorm().sqrt()

    fun bind() = Position2dDual(x, y)

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
        Vector2dDual(x.reparam(oldParam), y.reparam(oldParam))

    fun drop(n: Int) = Vector2dDual(x.drop(n), y.drop(n))
    fun value() = Vector2d(x.value(), y.value())
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
        require(real.size == imag.size)
        require(real.size <= 3)
    }

    companion object {
        @JvmStatic
        fun <Param> exp(theta: DualNum<Param>) = Rotation2dDual(theta.cos(), theta.sin())

        @JvmStatic
        fun <Param> constant(r: Rotation2d, n: Int) =
            Rotation2dDual<Param>(DualNum.constant(r.real, n), DualNum.constant(r.imag, n))
    }

    operator fun plus(x: Double) = this * Rotation2d.exp(x)
    operator fun plus(d: DualNum<Param>) = this * exp(d)
    operator fun minus(r: Rotation2dDual<Param>) = (r.inverse() * this).log()

    operator fun times(t: Twist2dDual<Param>) = Twist2dDual(this * t.transVel, t.rotVel)
    operator fun times(v: Vector2dDual<Param>) = Vector2dDual(real * v.x - imag * v.y, imag * v.x + real * v.y)
    operator fun times(v: Vector2d) = Vector2dDual(real * v.x - imag * v.y, imag * v.x + real * v.y)
    operator fun times(r: Rotation2dDual<Param>) =
        Rotation2dDual(real * r.real - imag * r.imag, real * r.imag + imag * r.real)
    operator fun times(r: Rotation2d) =
        Rotation2dDual(real * r.real - imag * r.imag, real * r.imag + imag * r.real)
    operator fun times(t: Transform2d) = Transform2dDual(this * t.trans, this * t.rot)

    fun inverse() = Rotation2dDual(real, -imag)

    // TODO: I'd like to somehow merge this with velocity()
    fun log() = DualNum<Param>(
        DoubleArray(size) {
            when (it) {
                0 -> atan2(imag[0], real[0])
                1 -> real[0] * imag[1] - imag[0] * real[1]
                2 -> real[0] * imag[2] - imag[0] * real[2]
                // ensured by init{} check
                else -> throw AssertionError()
            }
        }
    )

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
        Rotation2dDual(real.reparam(oldParam), imag.reparam(oldParam))

    // derivative of atan2 under unit norm assumption
    fun velocity() = real * imag.drop(1) - imag * real.drop(1)
    fun value() = Rotation2d(real.value(), imag.value())

    // TODO: turn into method?
    val size get() = real.size
}

/**
 * @usesMathJax
 *
 * 2D rigid transform comprised of [rot] followed by [trans].
 *
 * Transform names should take the form `txDestSource` to denote that the transform turns positions, vectors, twists,
 * etc. expressed in frame `Source` into frame `Dest`. This convention should extend to transformation (overloads of
 * [times]) inputs and outputs with the last word in a variable name indicating the frame the quantity is expressed in.
 * For example, the property `xDest = txDestSource * xSource` for all quantities `x` of any supported type.
 *
 * Transforms named `txWorldSource` for any frame `Source` are referred to as poses and are commonly abbreviated as
 * `sourcePose`.
 *
 * Advanced: Transforms in two dimensions comprise a Lie group referred to as SE(2). The terminology [exp] and [log]
 * comes from the Lie theory, and [this paper](https://arxiv.org/abs/1812.01537) gives a targeted exposition of the key
 * fundamentals.
 */
data class Transform2d(
    @JvmField
    val trans: Vector2d,
    @JvmField
    val rot: Rotation2d,
) {
    constructor(trans: Vector2d, rot: Double) : this(trans, Rotation2d.exp(rot))
    constructor(transX: Double, transY: Double, rot: Double) : this(Vector2d(transX, transY), rot)

    companion object {
        // see (133), (134) in https://ethaneade.com/lie.pdf
        // TODO: is this necessary?
        internal fun entries(theta: Double): Pair<Double, Double> {
            val u = theta + epsCopySign(theta)
            return Pair(
                sin(u) / u,
                (1.0 - cos(u)) / u
            )
        }

        @JvmStatic
        fun exp(incr: Twist2dIncrement): Transform2d {
            val rotation = Rotation2d.exp(incr.rotIncr)

            val (A, B) = entries(incr.rotIncr)
            val translation = Vector2d(
                A * incr.transIncr.x - B * incr.transIncr.y,
                B * incr.transIncr.x + A * incr.transIncr.y
            )

            return Transform2d(translation, rotation)
        }
    }

    operator fun plus(t: Twist2dIncrement) = this * exp(t)
    fun minusExp(t: Transform2d) = t.inverse() * this
    operator fun minus(t: Transform2d) = minusExp(t).log()

    operator fun times(t: Transform2d) = Transform2d(rot * t.trans + trans, rot * t.rot)
    operator fun times(v: Vector2d) = rot * v + trans
    operator fun times(t: Twist2d) = Twist2d(rot * t.transVel, t.rotVel)

    fun inverse() = Transform2d(rot.inverse() * -trans, rot.inverse())

    fun log(): Twist2dIncrement {
        val theta = rot.log()

        val (A, B) = entries(theta)
        val denom = Vector2d(A, B).sqrNorm()

        val (x, y) = trans
        return Twist2dIncrement(
            Vector2d(
                (A * x + B * y) / denom,
                (-B * x + A * y) / denom,
            ),
            theta,
        )
    }
}

/**
 * Dual version of [Transform2d].
 */
data class Transform2dDual<Param>(
    @JvmField
    val trans: Vector2dDual<Param>,
    @JvmField
    val rot: Rotation2dDual<Param>,
) {
    companion object {
        @JvmStatic
        fun <Param> constant(t: Transform2d, n: Int) =
            Transform2dDual<Param>(Vector2dDual.constant(t.trans, n), Rotation2dDual.constant(t.rot, n))
    }

    operator fun plus(t: Twist2dIncrement) = this * Transform2d.exp(t)

    operator fun times(t: Transform2d) = Transform2dDual(rot * t.trans + trans, rot * t.rot)
    operator fun times(t: Transform2dDual<Param>) = Transform2dDual(rot * t.trans + trans, rot * t.rot)
    operator fun times(t: Twist2dDual<Param>) = Twist2dDual(rot * t.transVel, t.rotVel)

    fun inverse() = rot.inverse().let {
        Transform2dDual(it * -trans, it)
    }

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
        Transform2dDual(trans.reparam(oldParam), rot.reparam(oldParam))

    fun value() = Transform2d(trans.value(), rot.value())
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

data class Twist2dIncrement(@JvmField val transIncr: Vector2d, @JvmField val rotIncr: Double)

data class Twist2dIncrementDual<Param>(
    @JvmField val transIncr: Vector2dDual<Param>,
    @JvmField val rotIncr: DualNum<Param>
) {
    fun value() = Twist2dIncrement(transIncr.value(), rotIncr.value())
    fun velocity() = Twist2dDual(transIncr.drop(1), rotIncr.drop(1))
}
