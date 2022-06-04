package com.acmerobotics.roadrunner

// TODO: why not Point2?
data class Position2<N : Num<N>>(val x: N, val y: N) {
    companion object {
        fun <Param> constant(x: Double, y: Double, n: Int) = Position2<DualNum<Param>>(
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
//    operator fun <End> minus(other: Position2<Origin, End, N>) = Vector2(x - other.x, y - other.y)
//    operator fun <End> plus(diff: Vector2<N>) = Position2<Origin, End, N>(x + diff.x, y + diff.y)
//
//    infix fun <OtherPoint> distTo(other: Position2<Origin, OtherPoint, N>) = (this - other).norm()
//
    fun free() = Vector2(x, y)
}

fun <Param> Position2<DualNum<Param>>.tangent() = Rotation2(x.drop(1), y.drop(1))

fun <Param, NewParam> Position2<DualNum<Param>>.reparam(oldParam: DualNum<NewParam>) =
    Position2(x.reparam(oldParam), y.reparam(oldParam))

//fun <Param> Position2<DualNum<Param>>.constant() = Position2(x.constant(), y.constant())

data class Vector2<N : Num<N>>(val x: N, val y: N) {
    operator fun plus(other: Vector2<N>) = Vector2(x + other.x, y + other.y)
    operator fun unaryMinus() = Vector2(-x, -y)

    infix fun dot(other: Vector2<N>) = x * other.x + y * other.y
    fun sqrNorm() = this dot this
    fun norm() = sqrNorm().sqrt()
}

fun <Param> Vector2<DualNum<Param>>.drop(n: Int) = Vector2(x.drop(n), y.drop(n))

fun <Param> Vector2<DualNum<Param>>.constant() = Vector2(x.constant(), y.constant())

class Rotation2<N : Num<N>>(val real: N, val imag: N) {
    operator fun times(vector: Vector2<N>) = Vector2(
        real * vector.x - imag * vector.y,
        imag * vector.x + real * vector.y
    )

    operator fun times(other: Rotation2<N>) = Rotation2(
        real * other.real - imag * other.imag,
        real * other.imag + imag * other.real
    )

    fun inverse() = Rotation2(real, -imag)
}

// TODO: is deriv() not more appropriate?
// I slightly prefer velocity because it emphasizes the difference between a 2d rotation matrix and a scalar angular velocity
fun <Param> Rotation2<DualNum<Param>>.velocity() =
    real * imag.drop(1) + real.drop(1) * imag

fun <Param> Rotation2<DualNum<Param>>.constant() = Rotation2(real.constant(), imag.constant())

class Transform2<N : Num<N>>(
    val rotation: Rotation2<N>,
    val translation: Vector2<N>
) {
    operator fun times(other: Transform2<N>): Transform2<N> =
        Transform2(rotation * other.rotation, rotation * other.translation + translation)

    fun inverse() = Transform2<N>(rotation.inverse(), rotation.inverse() * -translation)
}

// TODO: is this proper?
fun <Param> Transform2<DualNum<Param>>.velocity() = Twist2(translation.drop(1), rotation.velocity())

fun <Param> Transform2<DualNum<Param>>.constant() = Transform2(rotation.constant(), translation.constant())

// TODO: what goes inside? a rotation velocity
class Twist2<N : Num<N>>(val transVel: Vector2<N>, val rotVel: N)
//class TwistIncr2<N : Num<N>>(val translation: Vector2<N>, val rotation: N)
