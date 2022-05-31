package com.acmerobotics.roadrunner

interface Num<N> {
    operator fun plus(other: N): N
    operator fun minus(other: N): N
    operator fun times(other: N): N
    operator fun unaryMinus(): N

    fun sqrt(): N

    operator fun times(other: Double): N
}

// TODO: specialization of the geometry classes seems inevitable
@JvmInline
value class DoubleNum(val value: Double) : Num<DoubleNum> {
    override fun plus(other: DoubleNum) = DoubleNum(value + other.value)
    override fun minus(other: DoubleNum) = DoubleNum(value - other.value)
    override fun times(other: DoubleNum) = DoubleNum(value * other.value)
    override fun unaryMinus() = DoubleNum(-value)

    override fun sqrt() = DoubleNum(kotlin.math.sqrt(value))

    override fun times(other: Double) = DoubleNum(value * other)
}

class DualNum<Param>(val values: DoubleArray) : Num<DualNum<Param>> {
    companion object {
        fun <Param> constant(x: Double, n: Int) = DualNum<Param>(DoubleArray(n) {
            when (it) {
                0 -> x
                else -> 0.0
            }
        })
    }

    init {
        require(values.size <= 4)
    }

    // TODO: do we need a more efficient version?
    fun drop(n: Int) = DualNum<Param>(DoubleArray(values.size - n) { values[it + n] })

    fun constant() = DoubleNum(values.first())

    override fun plus(other: DualNum<Param>): DualNum<Param> {
        require(values.size == other.values.size)

        val out = DualNum<Param>(DoubleArray(values.size))
        for (i in out.values.indices) {
            out.values[i] = values[i] + other.values[i]
        }

        return out
    }

    override fun minus(other: DualNum<Param>): DualNum<Param> {
        require(values.size == other.values.size)

        val out = DualNum<Param>(DoubleArray(values.size))
        for (i in out.values.indices) {
            out.values[i] = values[i] - other.values[i]
        }

        return out
    }

    override fun times(other: DualNum<Param>): DualNum<Param> {
        require(values.size == other.values.size)

        val out = DualNum<Param>(DoubleArray(values.size))
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


    override fun unaryMinus(): DualNum<Param> {
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

    override fun sqrt(): DualNum<Param> {
        TODO("Not yet implemented")
    }

    override fun times(other: Double): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(values.size))
        for (i in out.values.indices) {
            out.values[i] = values[i] * other
        }

        return out
    }

    fun <NewParam> reparam(oldParam: DualNum<NewParam>): DualNum<NewParam> = TODO()
}
