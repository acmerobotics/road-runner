package com.acmerobotics.roadrunner

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

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
            Position2Dual(x.reparam(oldParam), y.reparam(oldParam))
}

// TODO: units?
data class Vector2(val x: Double, val y: Double) {
    operator fun plus(other: Vector2) = Vector2(x + other.x, y + other.y)
    operator fun unaryMinus() = Vector2(-x, -y)

    operator fun div(other: Double) = Vector2(x / other, y / other)

    infix fun dot(other: Vector2) = x * other.x + y * other.y
    fun sqrNorm() = this dot this
    fun norm() = sqrt(sqrNorm())
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
}


class Rotation2(val real: Double, val imag: Double) {
    operator fun times(vector: Vector2) = Vector2(
        real * vector.x - imag * vector.y,
        imag * vector.x + real * vector.y
    )

    operator fun times(other: Rotation2) = Rotation2(
        real * other.real - imag * other.imag,
        real * other.imag + imag * other.real
    )

    fun inverse() = Rotation2(real, -imag)
}

class Rotation2Dual<Param>(val real: DualNum<Param>, val imag: DualNum<Param>) {
    operator fun times(vector: Vector2Dual<Param>) = Vector2Dual<Param>(
            real * vector.x - imag * vector.y,
            imag * vector.x + real * vector.y
    )

    operator fun times(other: Rotation2Dual<Param>) = Rotation2Dual<Param>(
            real * other.real - imag * other.imag,
            real * other.imag + imag * other.real
    )

    fun inverse() = Rotation2Dual(real, -imag)

    // TODO: is deriv() not more appropriate?
    // I slightly prefer velocity because it emphasizes the difference between a 2d rotation matrix and a scalar angular velocity
    fun velocity() = real * imag.drop(1) + real.drop(1) * imag

    fun constant() = Rotation2(real.constant(), imag.constant())
}

class Transform2(
        val translation: Vector2,
        val rotation: Rotation2,
) {
    operator fun times(other: Transform2) =
        Transform2(rotation * other.translation + translation, rotation * other.rotation)

    fun inverse() = Transform2(rotation.inverse() * -translation, rotation.inverse())
}

class Transform2Dual<Param>(
        val translation: Vector2Dual<Param>,
        val rotation: Rotation2Dual<Param>,
) {
    operator fun times(other: Transform2Dual<Param>) =
            Transform2Dual(rotation * other.translation + translation, rotation * other.rotation)

    fun inverse() = Transform2Dual(rotation.inverse() * -translation, rotation.inverse())

    fun velocity() = Twist2Dual(translation.drop(1), rotation.velocity())

    fun constant() = Transform2(translation.constant(), rotation.constant())
}

// TODO: what goes inside? a rotation velocity
class Twist2(val transVel: Vector2, val rotVel: Double)
class Twist2Dual<Param>(val transVel: Vector2Dual<Param>, val rotVel: DualNum<Param>)

//class TwistIncr2<N : Num<N>>(val translation: Vector2<N>, val rotation: N)
