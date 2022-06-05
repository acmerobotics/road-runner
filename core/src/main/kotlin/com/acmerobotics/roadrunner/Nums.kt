package com.acmerobotics.roadrunner

import kotlin.math.min

// TODO: these all need tests of course
class DualNum<Param>(val values: DoubleArray) {
    companion object {
        fun <Param> constant(x: Double, n: Int) = DualNum<Param>(DoubleArray(n) {
            when (it) {
                0 -> x
                else -> 0.0
            }
        })

        fun <Param> variable(x: Double, n: Int) = DualNum<Param>(DoubleArray(n) {
            when (it) {
                0 -> x
                1 -> 1.0
                else -> 0.0
            }
        })
    }

    init {
        require(values.size <= 4)
    }

    // TODO: do we need a more efficient version?
    fun drop(n: Int) = DualNum<Param>(DoubleArray(values.size - n) { values[it + n] })

    fun constant() = values.first()

    operator fun plus(other: DualNum<Param>): DualNum<Param> {
        require(values.size == other.values.size)

        val out = DualNum<Param>(DoubleArray(values.size))
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
        if (out.values.size == 1) return out

        out.values[1] = values[0] * other.values[1] + values[1] * other.values[0]
        if (out.values.size == 2) return out

        out.values[2] = values[0] * other.values[2] + values[2] * other.values[0] +
            2 * values[1] * other.values[1]
        if (out.values.size == 3) return out

        out.values[3] = values[0] * other.values[3] + values[0] * other.values[3] +
            3 * (values[2] * other.values[1] + values[1] * other.values[2])
        return out
    }


    operator fun unaryMinus(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(values.size))
        for (i in out.values.indices) {
            out.values[i] = -values[i]
        }

        return out
    }

    fun recip(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(values.size))
        if (out.values.isEmpty()) return out

        val recip = 1.0 / values[0]
        out.values[0] = recip
        if (out.values.size == 1) return out

        val negRecip = -recip
        val negRecip2 = recip * negRecip
        val deriv = negRecip2 * values[1]
        out.values[1] = deriv
        if (out.values.size == 2) return out

        val int1 = 2 * negRecip * deriv
        val deriv2 = int1 * values[1] + negRecip2 * values[2]
        out.values[2] = deriv2
        if (out.values.size == 3) return out

        val int2 = int1 * values[2]
        out.values[3] =
            int2 + negRecip2 * values[3] +
                int2 - 2 * (deriv * deriv + recip * deriv2) * values[1]
        return out
    }

    fun sqrt(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(values.size))
        if (out.values.isEmpty()) return out

        val sqrt = kotlin.math.sqrt(values[0])
        out.values[0] = sqrt
        if (out.values.size == 1) return out

        val recip = 1 / (2 * sqrt)
        val deriv = recip * values[1]
        out.values[1] = deriv
        if (out.values.size == 2) return out

        val negRecip = -2 * recip
        val negRecip2 = recip * negRecip
        val int1 = negRecip2 * deriv
        val secondDeriv = int1 * values[1] + recip * values[2]
        out.values[2] = secondDeriv
        if (out.values.size == 3) return out

        val int2 = 2 * int1
        val thirdDeriv = recip * values[3] + int2 * values[2] +
                (deriv * negRecip * int2 +
                negRecip2 * int1) * values[1]
        out.values[3] = thirdDeriv

        return out
    }

    fun sin(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(values.size))
        if (out.values.isEmpty()) return out

        val sin = kotlin.math.sin(values[0])
        out.values[0] = sin
        if (out.values.size == 1) return out

        val cos = kotlin.math.cos(values[0])
        val deriv = cos * values[1]
        out.values[1] = deriv
        if (out.values.size == 2) return out

        val inDeriv2 = values[1] * values[1]
        out.values[2] = cos * values[2] - sin * inDeriv2
        if (out.values.size == 3) return out

        out.values[3] = cos * values[3] -
                3 * sin * values[1] * values[2] +
                deriv * inDeriv2

        return out
    }

    fun cos(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(values.size))
        if (out.values.isEmpty()) return out

        val cos = kotlin.math.cos(values[0])
        out.values[0] = cos
        if (out.values.size == 1) return out

        val sin = kotlin.math.sin(values[0])
        val negInDeriv = -values[1]
        val deriv = sin * negInDeriv
        out.values[1] = deriv
        if (out.values.size == 2) return out

        val int = cos * negInDeriv
        out.values[2] = int * values[1] - sin * values[2]
        if (out.values.size == 3) return out

        out.values[3] = deriv * negInDeriv * values[1] +
                3 * int * values[2] -
                sin * values[3]

        return out
    }

    operator fun plus(other: Double): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(values.size))
        for (i in out.values.indices) {
            out.values[i] = values[i] + other
        }

        return out
    }

    operator fun times(other: Double): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(values.size))
        for (i in out.values.indices) {
            out.values[i] = values[i] * other
        }

        return out
    }

    operator fun div(other: Double): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(values.size))
        for (i in out.values.indices) {
            out.values[i] = values[i] / other
        }

        return out
    }

    operator fun times(other: Vector2) =
            Vector2Dual(this * other.x, this * other.y)

    fun <NewParam> reparam(oldParam: DualNum<NewParam>): DualNum<NewParam> {
        // TODO: min() should probably be standard here
        val out = DualNum<NewParam>(DoubleArray(min(size, oldParam.size)))
        if (out.values.isEmpty()) return out

        out.values[0] = values[0]
        if (out.values.size == 1) return out

        out.values[1] = values[1] * oldParam[1]
        if (out.values.size == 2) return out

        val oldDeriv2 = oldParam.values[1] * oldParam.values[1]
        out.values[2] = oldDeriv2 * values[2] + oldParam.values[2] * values[1]
        if (out.values.size == 3) return out

        out.values[3] = values[1] * oldParam.values[3] +
                (3 * values[2] * oldParam.values[2] + values[3] * oldDeriv2) * oldParam.values[1]

        return out
    }

    operator fun get(i: Int) = values[i]

    val size get() = values.size
}
