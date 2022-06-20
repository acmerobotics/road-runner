package com.acmerobotics.roadrunner

import kotlin.math.abs
import kotlin.math.min

// TODO: figure out how to represent this
data class DualNum<Param>(val values: DoubleArray) {
    companion object {
        @JvmStatic
        fun <Param> constant(x: Double, n: Int) = DualNum<Param>(
            DoubleArray(n) {
                when (it) {
                    0 -> x
                    else -> 0.0
                }
            }
        )

        @JvmStatic
        fun <Param> variable(x: Double, n: Int) = DualNum<Param>(
            DoubleArray(n) {
                when (it) {
                    0 -> x
                    1 -> 1.0
                    else -> 0.0
                }
            }
        )
    }

    val size get() = values.size

    init {
        require(size <= 4)
    }

    fun value() = values.first()
    operator fun get(i: Int) = values[i]

    fun addFirst(x: Double) = DualNum<Param>(
        DoubleArray(size + 1) {
            if (it == 0) {
                x
            } else {
                values[it - 1]
            }
        }
    )

    fun drop(n: Int) = DualNum<Param>(DoubleArray(size - n) { values[it + n] })

    operator fun plus(other: DualNum<Param>): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(min(size, other.size)))
        for (i in out.values.indices) {
            out.values[i] = values[i] + other.values[i]
        }

        return out
    }

    operator fun minus(other: DualNum<Param>): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(min(size, other.size)))
        for (i in out.values.indices) {
            out.values[i] = values[i] - other.values[i]
        }

        return out
    }

    operator fun times(other: DualNum<Param>): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(min(size, other.size)))
        if (out.values.isEmpty()) return out

        out.values[0] = values[0] * other.values[0]
        if (out.size == 1) return out

        out.values[1] = values[0] * other.values[1] + values[1] * other.values[0]
        if (out.size == 2) return out

        out.values[2] = values[0] * other.values[2] + values[2] * other.values[0] +
            2 * values[1] * other.values[1]
        if (out.size == 3) return out

        out.values[3] = values[0] * other.values[3] + values[3] * other.values[0] +
            3 * (values[2] * other.values[1] + values[1] * other.values[2])
        return out
    }

    operator fun unaryMinus(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(size))
        for (i in out.values.indices) {
            out.values[i] = -values[i]
        }

        return out
    }

    fun recip(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(size))
        if (out.values.isEmpty()) return out

        val recip = 1.0 / values[0]
        out.values[0] = recip
        if (out.size == 1) return out

        val negRecip = -recip
        val negRecip2 = recip * negRecip
        val deriv = negRecip2 * values[1]
        out.values[1] = deriv
        if (out.size == 2) return out

        val int1 = 2 * negRecip * deriv
        val deriv2 = int1 * values[1] + negRecip2 * values[2]
        out.values[2] = deriv2
        if (out.size == 3) return out

        val int2 = int1 * values[2]
        out.values[3] =
            int2 + negRecip2 * values[3] +
            int2 - 2 * (deriv * deriv + recip * deriv2) * values[1]
        return out
    }

    operator fun div(other: DualNum<Param>) = this * other.recip()

    fun sqrt(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(size))
        if (out.values.isEmpty()) return out

        val sqrt = kotlin.math.sqrt(values[0])
        out.values[0] = sqrt
        if (out.size == 1) return out

        val recip = 1 / (2 * sqrt)
        val deriv = recip * values[1]
        out.values[1] = deriv
        if (out.size == 2) return out

        val negRecip = -2 * recip
        val negRecip2 = recip * negRecip
        val int1 = negRecip2 * deriv
        val secondDeriv = int1 * values[1] + recip * values[2]
        out.values[2] = secondDeriv
        if (out.size == 3) return out

        val int2 = 2 * int1
        out.values[3] = recip * values[3] + int2 * values[2] +
            (deriv * negRecip * int2 + negRecip2 * secondDeriv) * values[1]

        return out
    }

    fun sin(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(size))
        if (out.values.isEmpty()) return out

        val sin = kotlin.math.sin(values[0])
        out.values[0] = sin
        if (out.size == 1) return out

        val cos = kotlin.math.cos(values[0])
        val deriv = cos * values[1]
        out.values[1] = deriv
        if (out.size == 2) return out

        val inDeriv2 = values[1] * values[1]
        out.values[2] = cos * values[2] - sin * inDeriv2
        if (out.size == 3) return out

        out.values[3] = cos * values[3] -
            3 * sin * values[1] * values[2] -
            deriv * inDeriv2

        return out
    }

    fun cos(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(size))
        if (out.values.isEmpty()) return out

        val cos = kotlin.math.cos(values[0])
        out.values[0] = cos
        if (out.size == 1) return out

        val sin = kotlin.math.sin(values[0])
        val negInDeriv = -values[1]
        val deriv = sin * negInDeriv
        out.values[1] = deriv
        if (out.size == 2) return out

        val int = cos * negInDeriv
        out.values[2] = int * values[1] - sin * values[2]
        if (out.size == 3) return out

        out.values[3] = deriv * negInDeriv * values[1] +
            3 * int * values[2] -
            sin * values[3]

        return out
    }

    fun <NewParam> reparam(oldParam: DualNum<NewParam>): DualNum<NewParam> {
        val out = DualNum<NewParam>(DoubleArray(min(size, oldParam.size)))
        if (out.values.isEmpty()) return out

        out.values[0] = values[0]
        if (out.size == 1) return out

        out.values[1] = values[1] * oldParam[1]
        if (out.size == 2) return out

        val oldDeriv2 = oldParam.values[1] * oldParam.values[1]
        out.values[2] = oldDeriv2 * values[2] + oldParam.values[2] * values[1]
        if (out.size == 3) return out

        out.values[3] = values[1] * oldParam.values[3] +
            (3 * values[2] * oldParam.values[2] + values[3] * oldDeriv2) * oldParam.values[1]

        return out
    }

    operator fun plus(other: Double) = DualNum<Param>(
        DoubleArray(size) {
            when (it) {
                0 -> values[0] + other
                else -> values[it]
            }
        }
    )

    operator fun times(other: Double): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(size))
        for (i in out.values.indices) {
            out.values[i] = values[i] * other
        }

        return out
    }

    operator fun div(other: Double): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(size))
        for (i in out.values.indices) {
            out.values[i] = values[i] / other
        }

        return out
    }

    operator fun times(other: Vector2) =
        Vector2Dual(this * other.x, this * other.y)
}
