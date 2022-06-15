package com.acmerobotics.roadrunner

import kotlin.math.sin


val EPS = 2.2e-15

fun epsCopySign(x: Double) =
    if (x >= 0.0) {
        EPS
    } else {
        -EPS
    }

// TODO: putting this here because Ramsete needs it (maybe premature)
//fun sinc(x: Double): Double {
//    val u = x + epsCopySign(x)
//    return sin(u) / u
//}

fun clamp(x: Double, lo: Double, hi: Double): Double {
    if (x < lo) {
        return lo
    }
    if (x > hi) {
        return hi
    }
    return x
}

data class Interval(val min: Double, val max: Double) {
    fun pair() = Pair(min, max)
}

fun range(begin: Double, end: Double, samples: Int): List<Double> {
    require(samples >= 2)
    val dx = (end - begin) / (samples - 1)
    return (0 until samples).map { begin + dx * it }
}

fun rangeMiddle(begin: Double, end: Double, samples: Int): List<Double> {
    require(samples >= 1)
    val dx = (end - begin) / (samples - 1)
    return (0 until samples).map { begin + 0.5 * dx + dx * it }
}

fun lerp(x: Double, fromLo: Double, fromHi: Double, toLo: Double, toHi: Double) =
    toLo + (x - fromLo) * (toHi - toLo) / (fromHi - fromLo)

data class ScanResult(
    val values: List<Double>,
    val sums: List<Double>,
)

fun integralScan(a: Double, b: Double, eps: Double, f: (Double) -> Double): ScanResult {
    val m = (a + b) / 2
    val fa = f(a); val fm = f(m); val fb = f(b)

    var i = (b - a) / 8 * (
            fa + fm + fb +
                    f(a + 0.9501 * (b - a)) +
                    f(a + 0.2311 * (b - a)) +
                    f(a + 0.6068 * (b - a)) +
                    f(a + 0.4860 * (b - a)) +
                    f(a + 0.8913 * (b - a))
            )
    if (i == 0.0) {
        i = b - a
    }
    i *= eps / Math.ulp(1.0)

    val values = mutableListOf(0.0)
    val sums = mutableListOf(0.0)

    fun helper(a: Double, m: Double, b: Double, fa: Double, fm: Double, fb: Double) {
        val h = (b - a) / 4
        val ml = a + h; val mr = b - h
        val fml = f(ml); val fmr = f(mr)
        var i1 = h / 1.5 * (fa + 4 * fm + fb)
        val i2 = h / 3 * (fa + 4 * (fml + fmr) + 2 * fm + fb)
        i1 = (16 * i2 - i1) / 15
        if (i + (i1 - i2) == i || m <= a || b <= m) {
            values.add(b)
            sums.add(sums.last() + i1)
        } else {
            helper(a, ml, m, fa, fml, fm)
            helper(m, mr, b, fm, fmr, fb)
        }
    }

    helper(a, m, b, fa, fm, fb)

    return ScanResult(values, sums)
}
