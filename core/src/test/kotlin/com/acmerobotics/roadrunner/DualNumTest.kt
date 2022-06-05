package com.acmerobotics.roadrunner

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.random.Random
import kotlin.test.Test
import kotlin.test.assertEquals

class RefDualNum<Param>(
        val values: List<Double>
) {
    companion object {
        fun <Param> constant(x: Double, n: Int): RefDualNum<Param> =
                if (n <= 1) RefDualNum(listOf(x))
                else RefDualNum(listOf(x) + List(n - 1) { 0.0 })

        fun <Param> variable(x: Double, n: Int): RefDualNum<Param> =
                when {
                    n <= 1 -> RefDualNum(listOf(x))
                    n == 2 -> RefDualNum(listOf(x, 1.0))
                    else -> RefDualNum(listOf(x, 1.0) + List(n - 2) { 0.0 })
                }
    }

    constructor(head: Double, tail: RefDualNum<Param>) : this(listOf(head) + tail.values)

    fun constantLike(x: Double) = constant<Param>(x, values.size)

    fun drop(n: Int) = RefDualNum<Param>(values.drop(n))

    operator fun plus(other: RefDualNum<Param>) =
            RefDualNum<Param>(values.zip(other.values).map { it.first + it.second })

    operator fun minus(other: RefDualNum<Param>) =
            RefDualNum<Param>(values.zip(other.values).map { it.first - it.second })

    operator fun times(other: RefDualNum<Param>): RefDualNum<Param> =
            if (values.isEmpty() || other.values.isEmpty()) {
                RefDualNum(emptyList())
            } else {
                RefDualNum(
                        values.first() * other.values.first(),
                        this * other.drop(1) + drop(1) * other
                )
            }

    operator fun unaryMinus() = constantLike(0.0) - this

    private fun dropLast(n: Int) = RefDualNum<Param>(values.dropLast(n))

    fun lift(f: (Double) -> Double, df: RefDualNum<Param>.() -> RefDualNum<Param>): RefDualNum<Param> =
            if (values.isEmpty()) {
                RefDualNum(emptyList())
            } else {
                // dropLast() here ensures the recursive arguments decrease
                RefDualNum(f(values.first()), dropLast(1).df() * drop(1))
            }

    fun sqr() = this * this
    fun recip(): RefDualNum<Param> = lift({ 1.0 / it }) { -recip().sqr() }

    operator fun div(other: RefDualNum<Param>) = this * other.recip()

    fun sqrt(): RefDualNum<Param> = lift(::sqrt) { recip().sqrt() * 0.5 }

    fun sin(): RefDualNum<Param> = lift(::sin, RefDualNum<Param>::cos)
    fun cos(): RefDualNum<Param> = lift(::cos) { -sin() }

    fun <NewParam> reparam(oldParam: RefDualNum<NewParam>): RefDualNum<NewParam> =
            if (values.isEmpty()) {
                RefDualNum(emptyList())
            } else {
                RefDualNum(value(), drop(1).reparam(oldParam) * oldParam.drop(1))
            }

    operator fun plus(other: Double): RefDualNum<Param> = this + constantLike(other)
    operator fun times(other: Double): RefDualNum<Param> = this * constantLike(other)
    operator fun div(other: Double): RefDualNum<Param> = this / constantLike(other)

    fun value() = values.first()
}

object TestParam

fun assertDualEquals(expected: RefDualNum<TestParam>, actual: DualNum<TestParam>) {
    assertEquals(expected.values.size, actual.size)

    expected.values
            .zip(actual.values.toList())
            .forEach { (expected, actual) ->
                assertEquals(expected, actual, 1e-6)
            }
}

fun testRandomMonadic(
        expected: (RefDualNum<TestParam>) -> RefDualNum<TestParam>,
        actual: (DualNum<TestParam>) -> DualNum<TestParam>,
        n: Int = 100
) {
    val r = Random.Default
    repeat(n) {
        val values = listOf(r.nextDouble(), r.nextDouble(), r.nextDouble(), r.nextDouble())
        assertDualEquals(
                expected(RefDualNum(values)),
                actual(DualNum(values.toDoubleArray()))
        )
    }
}

class DualNumTest {
    @Test fun testRecip() = testRandomMonadic({ it.recip() }, { it.recip() })
    @Test fun testSqrt() = testRandomMonadic({ it.sqrt() }, { it.sqrt() })
    @Test fun testSin() = testRandomMonadic({ it.sin() }, { it.sin() })
    @Test fun testCos() = testRandomMonadic({ it.cos() }, { it.cos() })
}
