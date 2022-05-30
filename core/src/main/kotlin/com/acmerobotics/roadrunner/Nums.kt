package com.acmerobotics.roadrunner

import kotlin.math.*

interface Num<N> {

}

class DualNum<Param>(val values: DoubleArray) : Num<DualNum<Param>> {

}

//interface Num<N> {
//    operator fun minus(other: N): N
//    operator fun plus(other: N): N
//    operator fun times(other: N): N
//    operator fun div(other: N): N
//    operator fun unaryMinus(): N
//
//    fun sqrt(): N
//    fun atan(): N
//    fun asin(): N
//    fun cos(): N
//    fun sin(): N
//
//    fun value(): Double
//
//    operator fun plus(other: Double): N
//    operator fun times(other: Double): N
//    operator fun div(other: Double): N
//}
//
//operator fun <N : Num<N>> Double.minus(other: N) = -other + this
//operator fun <N : Num<N>> Double.times(other: N) = other * this
//
//data class DoubleNum(val value: Double) : Num<DoubleNum> {
//    override fun plus(other: DoubleNum) = DoubleNum(value + other.value)
//    override fun minus(other: DoubleNum) = DoubleNum(value - other.value)
//    override fun times(other: DoubleNum) = DoubleNum(value * other.value)
//    override fun div(other: DoubleNum) = DoubleNum(value / other.value)
//    override fun unaryMinus() = DoubleNum(-value)
//
//    override fun sqrt() = DoubleNum(sqrt(value))
//
//    override fun sin() = DoubleNum(sin(value))
//    override fun cos() = DoubleNum(cos(value))
//
//    override fun asin() = DoubleNum(asin(value))
//    override fun atan() = DoubleNum(atan(value))
//
//    override fun value() = value
//
//    override fun plus(other: Double) = DoubleNum(value + other)
//    override fun times(other: Double) = DoubleNum(value * other)
//    override fun div(other: Double) = DoubleNum(value / other)
//}
//
//data class DualNum<Param>(
//    val values: List<Double>
//) : Num<DualNum<Param>> {
//    companion object {
//        fun <Param> constant(x: Double, n: Int): DualNum<Param> =
//            if (n <= 1) DualNum(listOf(x))
//            else DualNum(listOf(x) + List(n - 1) { 0.0 })
//
//        fun <Param> variable(x: Double, n: Int): DualNum<Param> =
//            when {
//                n <= 1 -> DualNum(listOf(x))
//                n == 2 -> DualNum(listOf(x, 1.0))
//                else -> DualNum(listOf(x, 1.0) + List(n - 2) { 0.0 })
//            }
//    }
//
//    constructor(head: Double, tail: DualNum<Param>) : this(listOf(head) + tail.values)
//
//    fun drop(n: Int) = DualNum<Param>(values.drop(n))
//    fun take(n: Int) = DualNum<Param>(values.take(n))
//
//    fun constant() = DoubleNum(values.first())
//
//    private fun dropLast(n: Int) = DualNum<Param>(values.dropLast(n))
//
//    override operator fun plus(other: DualNum<Param>) =
//        DualNum<Param>(values.zip(other.values).map { it.first + it.second })
//
//    override operator fun minus(other: DualNum<Param>) =
//        DualNum<Param>(values.zip(other.values).map { it.first - it.second })
//
//    override operator fun times(other: DualNum<Param>): DualNum<Param> =
//        if (values.isEmpty() || other.values.isEmpty()) {
//            DualNum(emptyList())
//        } else {
//            DualNum(
//                values.first() * other.values.first(),
//                this * other.drop(1) + drop(1) * other
//            )
//        }
//
//    fun constantLike(x: Double) = constant<Param>(x, values.size)
//    fun variableLike(x: Double) = variable<Param>(x, values.size)
//
//    override operator fun times(other: Double): DualNum<Param> = this * constantLike(other)
//    override operator fun plus(other: Double): DualNum<Param> = this + constantLike(other)
//    operator fun minus(other: Double): DualNum<Param> = this - constantLike(other)
//    override operator fun div(other: Double): DualNum<Param> = this / constantLike(other)
//
//    fun lift(f: (Double) -> Double, df: DualNum<Param>.() -> DualNum<Param>): DualNum<Param> =
//        if (values.isEmpty()) {
//            DualNum(emptyList())
//        } else {
//            // dropLast() here ensures the recursive arguments decrease
//            DualNum(f(values.first()), dropLast(1).df() * drop(1))
//        }
//
//    fun recip(): DualNum<Param> = lift({ 1.0 / it }) { -recip().sqr() }
//    override operator fun div(other: DualNum<Param>) = this * other.recip()
//
//    override fun sqrt(): DualNum<Param> = lift(::sqrt) { 0.5 * recip().sqrt() }
//
//    override fun sin(): DualNum<Param> = lift(::sin, DualNum<Param>::cos)
//    override fun cos(): DualNum<Param> = lift(::cos) { -sin() }
//
//    override operator fun unaryMinus() = constantLike(0.0) - this
//
//    fun sqr() = this * this
//
//    override fun asin(): DualNum<Param> = lift(::asin) { (constantLike(1.0) - sqr()).sqrt().recip() }
//    override fun atan(): DualNum<Param> = lift(::atan) { (constantLike(1.0) + sqr()).recip() }
//
//    fun <NewParam> reparam(oldParam: DualNum<NewParam>): DualNum<NewParam> =
//        if (values.isEmpty()) {
//            DualNum(emptyList())
//        } else {
//            DualNum(value(), drop(1).reparam(oldParam) * oldParam.drop(1))
//        }
//
//    override fun value() = values.first()
//}
//
//operator fun <Param> Double.times(other: DualNum<Param>) = other * this
//operator fun <Param> Double.plus(other: DualNum<Param>): DualNum<Param> = other + this
//operator fun <Param> Double.minus(other: DualNum<Param>) = other.constantLike(this) - other
