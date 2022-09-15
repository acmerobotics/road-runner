@file:JvmName("Math")

package com.acmerobotics.roadrunner

// ~10 * machine epsilon
private const val EPS = 2.2e-15

/**
 * @usesMathJax
 *
 * Function \(snz(x)\) from section VI.A of the [SymForce paper](https://arxiv.org/abs/2204.07889) for use in
 * singularity handling.
 */
fun snz(x: Double) =
    if (x >= 0.0) {
        EPS
    } else {
        -EPS
    }

fun clamp(x: Double, lo: Double, hi: Double): Double {
    if (x < lo) {
        return lo
    }
    if (x > hi) {
        return hi
    }
    return x
}

data class MinMax(@JvmField val min: Double, @JvmField val max: Double)

/**
 * @usesMathJax
 *
 * Partitions \([a, b]\) into \((n - 1)\) equal intervals and returns the endpoints.
 *
 * @param[begin] \(a\)
 * @param[end] \(b\)
 * @param[samples] \(n\)
 */
fun range(begin: Double, end: Double, samples: Int): List<Double> {
    require(samples >= 2)
    val dx = (end - begin) / (samples - 1)
    return (0 until samples).map { begin + dx * it }
}

/**
 * @usesMathJax
 *
 * Partitions \([a, b]\) into \(n\) equal intervals and returns the center values.
 *
 * @param[begin] \(a\)
 * @param[end] \(b\)
 * @param[samples] \(n\)
 */
fun rangeMiddle(begin: Double, end: Double, samples: Int): List<Double> {
    require(samples >= 1)
    val dx = (end - begin) / samples
    return (0..samples).map { begin + 0.5 * dx + dx * it }
}

fun lerp(x: Double, fromLo: Double, fromHi: Double, toLo: Double, toHi: Double) =
    toLo + (x - fromLo) * (toHi - toLo) / (fromHi - fromLo)

data class IntegralScanResult(
    @JvmField
    val values: List<Double>,
    @JvmField
    val sums: List<Double>,
)

/**
 * @usesMathJax
 *
 * Returns samples of \(g(t) = \int_a^t f(x) \, dx\) for various values \(a \leq t \leq b\). The sampling points are
 * chosen adaptively using the algorithm `adaptsim` from [Gander and Gautschi](https://doi.org/10.1023/A:1022318402393)
 * ([more accessible link](https://users.wpi.edu/~walker/MA510/HANDOUTS/w.gander,w.gautschi,Adaptive_Quadrature,BIT_40,2000,84-101.pdf)).
 *
 * @param[a] \(a\)
 * @param[b] \(b\)
 * @param[f] \(f(x)\)
 * @param[eps] desired error in the length approximation \(g(b)\)
 */
fun integralScan(a: Double, b: Double, eps: Double, f: (Double) -> Double): IntegralScanResult {
    val m = (a + b) / 2
    val fa = f(a)
    val fm = f(m)
    val fb = f(b)

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
        val ml = a + h
        val mr = b - h
        val fml = f(ml)
        val fmr = f(mr)
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

    return IntegralScanResult(values, sums)
}
