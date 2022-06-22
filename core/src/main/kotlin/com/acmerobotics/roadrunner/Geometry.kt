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
data class Position2(@JvmField val x: Double, @JvmField val y: Double) {
    operator fun plus(v: Vector2) = Position2(x + v.x, y + v.y)
    operator fun minus(p: Position2) = Vector2(x - p.x, y - p.y)
}

/**
 * Dual version of [Position2].
 */
data class Position2Dual<Param>(@JvmField val x: DualNum<Param>, @JvmField val y: DualNum<Param>) {
    companion object {
        @JvmStatic
        fun <Param> constant(p: Position2, n: Int) = Position2Dual<Param>(
            DualNum.constant(p.x, n), DualNum.constant(p.y, n)
        )
    }

    operator fun minus(p: Position2Dual<Param>) = Vector2Dual(x - p.x, y - p.y)

    fun free() = Vector2Dual(x, y)

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) = Position2Dual(x.reparam(oldParam), y.reparam(oldParam))

    fun value() = Position2(x.value(), y.value())
    fun tangentVec() = Vector2Dual(x.drop(1), y.drop(1))
}

fun Position2Dual<Arclength>.tangent() = Rotation2Dual(x.drop(1), y.drop(1))

/**
 * @usesMathJax
 *
 * Vector \((x, y)\)
 */
data class Vector2(@JvmField val x: Double, @JvmField val y: Double) {
    operator fun plus(v: Vector2) = Vector2(x + v.x, y + v.y)
    operator fun minus(v: Vector2) = Vector2(x - v.x, y - v.y)
    operator fun unaryMinus() = Vector2(-x, -y)

    operator fun times(z: Double) = Vector2(x * z, y * z)
    operator fun div(z: Double) = Vector2(x / z, y / z)

    infix fun dot(v: Vector2) = x * v.x + y * v.y
    fun sqrNorm() = this dot this
    fun norm() = sqrt(sqrNorm())

    fun bind() = Position2(x, y)
}

/**
 * Dual version of [Vector2].
 */
data class Vector2Dual<Param>(@JvmField val x: DualNum<Param>, @JvmField val y: DualNum<Param>) {
    companion object {
        @JvmStatic
        fun <Param> constant(v: Vector2, n: Int) =
            Vector2Dual<Param>(DualNum.constant(v.x, n), DualNum.constant(v.y, n))
    }

    operator fun plus(v: Vector2) = Vector2Dual(x + v.x, y + v.y)
    operator fun plus(v: Vector2Dual<Param>) = Vector2Dual(x + v.x, y + v.y)
    operator fun plus(p: Position2) = Position2Dual(x + p.x, y + p.y)
    operator fun minus(v: Vector2Dual<Param>) = Vector2Dual(x - v.x, y - v.y)
    operator fun unaryMinus() = Vector2Dual(-x, -y)

    operator fun div(z: Double) = Vector2Dual(x / z, y / z)

    infix fun dot(v: Vector2Dual<Param>) = x * v.x + y * v.y
    fun sqrNorm() = this dot this
    fun norm() = sqrNorm().sqrt()

    fun bind() = Position2Dual(x, y)

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
        Vector2Dual(x.reparam(oldParam), y.reparam(oldParam))

    fun drop(n: Int) = Vector2Dual(x.drop(n), y.drop(n))
    fun value() = Vector2(x.value(), y.value())
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
data class Rotation2(@JvmField val real: Double, @JvmField val imag: Double) {
    companion object {
        /**
         * Turns an unnormalized angle into a rotation.
         */
        @JvmStatic
        fun exp(theta: Double) = Rotation2(cos(theta), sin(theta))
    }

    operator fun plus(x: Double) = this * exp(x)
    operator fun minus(r: Rotation2) = (r.inverse() * this).log()

    operator fun times(v: Vector2) = Vector2(real * v.x - imag * v.y, imag * v.x + real * v.y)
    operator fun times(t: Twist2) = Twist2(this * t.transVel, t.rotVel)
    operator fun times(r: Rotation2) =
        Rotation2(real * r.real - imag * r.imag, real * r.imag + imag * r.real)

    fun vec() = Vector2(real, imag)

    fun inverse() = Rotation2(real, -imag)

    /**
     * Get the rotation as a normalized [Double].
     */
    fun log() = atan2(imag, real)
}

/**
 * Dual version of [Rotation2].
 */
data class Rotation2Dual<Param>(@JvmField val real: DualNum<Param>, @JvmField val imag: DualNum<Param>) {
    init {
        require(real.size == imag.size)
        require(real.size <= 3)
    }

    companion object {
        fun <Param> exp(theta: DualNum<Param>) = Rotation2Dual(theta.cos(), theta.sin())

        fun <Param> constant(r: Rotation2, n: Int) =
            Rotation2Dual<Param>(DualNum.constant(r.real, n), DualNum.constant(r.imag, n))
    }

    operator fun plus(x: Double) = this * Rotation2.exp(x)
    operator fun plus(d: DualNum<Param>) = this * exp(d)
    operator fun minus(r: Rotation2Dual<Param>) = (r.inverse() * this).log()

    operator fun times(t: Twist2Dual<Param>) = Twist2Dual(this * t.transVel, t.rotVel)
    operator fun times(v: Vector2Dual<Param>) = Vector2Dual(real * v.x - imag * v.y, imag * v.x + real * v.y)
    operator fun times(v: Vector2) = Vector2Dual(real * v.x - imag * v.y, imag * v.x + real * v.y)
    operator fun times(r: Rotation2Dual<Param>) =
        Rotation2Dual(real * r.real - imag * r.imag, real * r.imag + imag * r.real)
    operator fun times(r: Rotation2) =
        Rotation2Dual(real * r.real - imag * r.imag, real * r.imag + imag * r.real)

    fun inverse() = Rotation2Dual(real, -imag)

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
        Rotation2Dual(real.reparam(oldParam), imag.reparam(oldParam))

    // derivative of atan2 under unit norm assumption
    fun velocity() = real * imag.drop(1) - imag * real.drop(1)
    fun value() = Rotation2(real.value(), imag.value())

    // TODO: turn into method?
    val size get() = real.size
}

/**
 * @usesMathJax
 *
 * 2D rigid transform comprised of [rot] followed by [trans].
 *
 * Advanced: Transforms in two dimensions comprise a Lie group referred to as SE(2). The terminology [exp] and [log]
 * comes from the Lie theory, and [this paper](https://arxiv.org/abs/1812.01537) gives a targeted exposition of the key
 * fundamentals.
 */
data class Transform2(
    @JvmField
    val trans: Vector2,
    @JvmField
    val rot: Rotation2,
) {
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
        fun exp(incr: Twist2Increment): Transform2 {
            val rotation = Rotation2.exp(incr.rotIncr)

            val (A, B) = entries(incr.rotIncr)
            val translation = Vector2(
                A * incr.transIncr.x - B * incr.transIncr.y,
                B * incr.transIncr.x + A * incr.transIncr.y
            )

            return Transform2(translation, rotation)
        }
    }

    operator fun plus(t: Twist2Increment) = this * exp(t)
    fun minusExp(t: Transform2) = t.inverse() * this
    operator fun minus(t: Transform2) = minusExp(t).log()

    operator fun times(t: Transform2) = Transform2(rot * t.trans + trans, rot * t.rot)
    operator fun times(v: Vector2) = rot * v + trans
    operator fun times(t: Twist2) = Twist2(rot * t.transVel, t.rotVel)

    fun inverse() = Transform2(rot.inverse() * -trans, rot.inverse())

    fun log(): Twist2Increment {
        val theta = rot.log()

        val (A, B) = entries(theta)
        val denom = Vector2(A, B).sqrNorm()

        val (x, y) = trans
        return Twist2Increment(
            Vector2(
                (A * x + B * y) / denom,
                (-B * x + A * y) / denom,
            ),
            theta,
        )
    }
}

/**
 * Dual version of [Transform2].
 */
data class Transform2Dual<Param>(
    @JvmField
    val trans: Vector2Dual<Param>,
    @JvmField
    val rot: Rotation2Dual<Param>,
) {
    companion object {
        @JvmStatic
        fun <Param> constant(t: Transform2, n: Int) =
            Transform2Dual<Param>(Vector2Dual.constant(t.trans, n), Rotation2Dual.constant(t.rot, n))
    }

    operator fun plus(t: Twist2Increment) = this * Transform2.exp(t)

    operator fun times(t: Transform2) = Transform2Dual(rot * t.trans + trans, rot * t.rot)
    operator fun times(t: Transform2Dual<Param>) = Transform2Dual(rot * t.trans + trans, rot * t.rot)
    operator fun times(t: Twist2Dual<Param>) = Twist2Dual(rot * t.transVel, t.rotVel)

    fun inverseThenTimes(t: Twist2Dual<Param>) = Twist2Dual(rot.inverse() * t.transVel, t.rotVel)

    fun inverse() = rot.inverse().let {
        Transform2Dual(it * -trans, it)
    }

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
        Transform2Dual(trans.reparam(oldParam), rot.reparam(oldParam))

    fun value() = Transform2(trans.value(), rot.value())
    fun velocity() = Twist2Dual(trans.drop(1), rot.velocity())
}

/**
 * 2D twist (velocities)
 */
data class Twist2(@JvmField val transVel: Vector2, @JvmField val rotVel: Double) {
    operator fun minus(t: Twist2) = Twist2(transVel - t.transVel, rotVel - t.rotVel)
}

/**
 * Dual version of [Twist2].
 */
data class Twist2Dual<Param>(@JvmField val transVel: Vector2Dual<Param>, @JvmField val rotVel: DualNum<Param>) {
    companion object {
        @JvmStatic
        fun <Param> constant(t: Twist2, n: Int) =
            Twist2Dual<Param>(Vector2Dual.constant(t.transVel, n), DualNum.constant(t.rotVel, n))
    }

    operator fun plus(other: Twist2) = Twist2Dual(transVel + other.transVel, rotVel + other.rotVel)

    fun value() = Twist2(transVel.value(), rotVel.value())
}

data class Twist2Increment(@JvmField val transIncr: Vector2, @JvmField val rotIncr: Double)

data class Twist2IncrementDual<Param>(
    @JvmField val transIncr: Vector2Dual<Param>,
    @JvmField val rotIncr: DualNum<Param>
) {
    fun value() = Twist2Increment(transIncr.value(), rotIncr.value())
    fun velocity() = Twist2Dual(transIncr.drop(1), rotIncr.drop(1))
}
