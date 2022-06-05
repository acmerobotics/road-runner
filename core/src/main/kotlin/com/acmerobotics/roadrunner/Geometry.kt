package com.acmerobotics.roadrunner

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

// TODO: why not Point2?
data class Position2(val x: Double, val y: Double) {
//
//        fun <Origin> origin() = Position2<Origin, Origin, DoubleNum>(DoubleNum(0.0), DoubleNum(0.0))
//        fun <Origin, Param> origin(n : Int) = Position2<Origin, Origin, DualNum<Param>>(DualNum.constant(0.0, n), DualNum.constant(0.0, n))
//
//        fun <Origin, Point, N : Num<N>> bind(v: Vector2<N>) = Position2<Origin, Point, N>(v.x, v.y)
//    }
//
    operator fun minus(other: Position2) = Vector2(x - other.x, y - other.y)
//    operator fun <End> plus(diff: Vector2<N>) = Position2<Origin, End, N>(x + diff.x, y + diff.y)
//
//    infix fun <OtherPoint> distTo(other: Position2<Origin, OtherPoint, N>) = (this - other).norm()
//
    fun free() = Vector2(x, y)
}

data class Position2Dual<Param>(val x: DualNum<Param>, val y: DualNum<Param>) {
    companion object {
        fun <Param> constant(x: Double, y: Double, n: Int) = Position2Dual<Param>(
                DualNum.constant(x, n), DualNum.constant(y, n)
        )
    }
    //
//        fun <Origin> origin() = Position2<Origin, Origin, DoubleNum>(DoubleNum(0.0), DoubleNum(0.0))
//        fun <Origin, Param> origin(n : Int) = Position2<Origin, Origin, DualNum<Param>>(DualNum.constant(0.0, n), DualNum.constant(0.0, n))
//
//        fun <Origin, Point, N : Num<N>> bind(v: Vector2<N>) = Position2<Origin, Point, N>(v.x, v.y)
//    }
//
    operator fun minus(other: Position2Dual<Param>) = Vector2Dual(x - other.x, y - other.y)
    //    operator fun <End> plus(diff: Vector2<N>) = Position2<Origin, End, N>(x + diff.x, y + diff.y)
//
//    infix fun <OtherPoint> distTo(other: Position2<Origin, OtherPoint, N>) = (this - other).norm()
//
    fun free() = Vector2Dual(x, y)

    fun tangent() = Rotation2Dual(x.drop(1), y.drop(1))
    fun tangentVec() = Vector2Dual(x.drop(1), y.drop(1))

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
            Position2Dual(x.reparam(oldParam), y.reparam(oldParam))

    fun constant() = Position2(x.constant(), y.constant())
}

// TODO: units?
data class Vector2(val x: Double, val y: Double) {
    operator fun plus(other: Vector2) = Vector2(x + other.x, y + other.y)
    operator fun unaryMinus() = Vector2(-x, -y)

    operator fun times(other: Double) = Vector2(x * other, y * other)
    operator fun div(other: Double) = Vector2(x / other, y / other)

    infix fun dot(other: Vector2) = x * other.x + y * other.y
    fun sqrNorm() = this dot this
    fun norm() = sqrt(sqrNorm())

    fun bind() = Position2(x, y)
}

data class Vector2Dual<Param>(val x: DualNum<Param>, val y: DualNum<Param>) {
    operator fun plus(other: Vector2Dual<Param>) = Vector2Dual(x + other.x, y + other.y)
    operator fun unaryMinus() = Vector2Dual(-x, -y)

    operator fun div(other: Double) = Vector2Dual(x / other, y / other)

    operator fun plus(other: Position2) = Position2Dual(x + other.x, y + other.y)

    infix fun dot(other: Vector2Dual<Param>) = x * other.x + y * other.y
    fun sqrNorm() = this dot this
    fun norm() = sqrNorm().sqrt()

    fun drop(n: Int) = Vector2Dual(x.drop(n), y.drop(n))
    fun constant() = Vector2(x.constant(), y.constant())

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
            Vector2Dual(x.reparam(oldParam), y.reparam(oldParam))

    fun bind() = Position2Dual(x, y)
}


class Rotation2(val real: Double, val imag: Double) {
    companion object {
        fun exp(theta: Double) = Rotation2(cos(theta), sin(theta))
    }

    operator fun times(vector: Vector2) = Vector2(
        real * vector.x - imag * vector.y,
        imag * vector.x + real * vector.y
    )

    operator fun times(other: Rotation2) = Rotation2(
        real * other.real - imag * other.imag,
        real * other.imag + imag * other.real
    )

    fun inverse() = Rotation2(real, -imag)

    fun <Param> constant(n: Int) =
            Rotation2Dual<Param>(DualNum.constant(real, n), DualNum.constant(imag, n))

    fun vec() = Vector2(real, imag)

    fun log() = atan2(imag, real)
}

class Rotation2Dual<Param>(val real: DualNum<Param>, val imag: DualNum<Param>) {
    init {
        require(real.size == imag.size)
        require(real.size <= 3)
    }

    companion object {
        fun <Param> exp(theta: DualNum<Param>) = Rotation2Dual<Param>(theta.cos(), theta.sin())
    }

//    operator fun plus(other: DualNum<Param>) = this * exp(other)
//    operator fun plus(other: Double) = this * exp(other)

    operator fun times(vector: Vector2Dual<Param>) = Vector2Dual(
            real * vector.x - imag * vector.y,
            imag * vector.x + real * vector.y
    )

    operator fun times(vector: Vector2) = Vector2Dual(
            real * vector.x - imag * vector.y,
            imag * vector.x + real * vector.y
    )

    operator fun times(other: Rotation2Dual<Param>) = Rotation2Dual(
            real * other.real - imag * other.imag,
            real * other.imag + imag * other.real
    )

    operator fun times(other: Rotation2) = Rotation2Dual(
            real * other.real - imag * other.imag,
            real * other.imag + imag * other.real
    )

    fun inverse() = Rotation2Dual(real, -imag)

    // TODO: is this subsumed by log()?
    // TODO: is deriv() not more appropriate?
    // I slightly prefer velocity because it emphasizes the difference between a 2d rotation matrix and a scalar angular velocity
    fun velocity() = real * imag.drop(1) + real.drop(1) * imag

    fun constant() = Rotation2(real.constant(), imag.constant())

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
            Rotation2Dual(real.reparam(oldParam), imag.reparam(oldParam))

    operator fun minus(other: Rotation2Dual<Param>) = (other.inverse() * this).log()

    fun log() = DualNum<Param>(DoubleArray(size) {
        when (it) {
            0 -> atan2(imag[0], real[0])
            1 -> real[0] * imag[1] - imag[0] * real[1]
            2 -> real[0] * imag[2] - imag[0] * real[2]
            // ensured by init{} check
            else -> throw AssertionError()
        }
    })

    val size get() = real.size
}

class Transform2(
        val translation: Vector2,
        val rotation: Rotation2,
) {
    companion object {
        // see (133), (134) in https://ethaneade.com/lie.pdf
        private fun entries(theta: Double) : Pair<Double, Double> {
            // TODO: better singularity math
//        val A = if (theta epsilonEquals 0.0) {
//            1.0 - theta * theta / 6.0
//        } else {
//            theta.sin() / theta
//        }
//        val B = if (theta.value() epsilonEquals 0.0) {
//            theta / 2.0
//        } else {
//            (1.0 - theta.cos()) / theta
//        }
            return Pair(sin(theta) / theta, (1.0 - cos(theta)) / theta)
        }

        fun exp(incr: Twist2Incr): Transform2 {
            val rotation = Rotation2.exp(incr.rotIncr)

            val (A, B) = entries(incr.rotIncr)
            val translation = Vector2(
                    A * incr.transIncr.x - B * incr.transIncr.y,
                    B * incr.transIncr.x + A * incr.transIncr.y
            )

            return Transform2(translation, rotation)
        }
    }

    operator fun times(other: Transform2) =
        Transform2(rotation * other.translation + translation, rotation * other.rotation)

    fun inverse() = Transform2(rotation.inverse() * -translation, rotation.inverse())

    fun log(): Twist2Incr {
        val theta = rotation.log()

        val (A, B) = entries(theta)
        val denom = Vector2(A, B).sqrNorm()

        val (x, y) = translation
        return Twist2Incr(
                Vector2(
                    (A * x + B * y) / denom,
                    (-B * x + A * y) / denom,
                ),
                theta,
        )
    }

    operator fun minus(other: Transform2) = (other.inverse() * this).log()
}

class Transform2Dual<Param>(
        val translation: Vector2Dual<Param>,
        val rotation: Rotation2Dual<Param>,
) {
    operator fun times(other: Transform2Dual<Param>) =
            Transform2Dual(rotation * other.translation + translation, rotation * other.rotation)

    operator fun times(other: Transform2) =
            Transform2Dual(rotation * other.translation + translation, rotation * other.rotation)

    // TODO: is this the right ordering?
    operator fun plus(other: Twist2Incr) = this * Transform2.exp(other)

    fun inverse() = Transform2Dual(rotation.inverse() * -translation, rotation.inverse())

    fun velocity() = Twist2Dual(translation.drop(1), rotation.velocity())

    fun constant() = Transform2(translation.constant(), rotation.constant())

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
            Transform2Dual(translation.reparam(oldParam), rotation.reparam(oldParam))
}

// TODO: what goes inside? a rotation velocity
class Twist2(val transVel: Vector2, val rotVel: Double)
class Twist2Dual<Param>(val transVel: Vector2Dual<Param>, val rotVel: DualNum<Param>)

class Twist2Incr(val transIncr: Vector2, val rotIncr: Double)
