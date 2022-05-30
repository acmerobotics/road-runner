package com.acmerobotics.roadrunner

import kotlin.math.sqrt

// TODO: is this okay being an object on the Java side?
object Internal

class QuinticSpline1(
    start: Double,
    startDeriv: Double,
    startSecondDeriv: Double,
    end: Double,
    endDeriv: Double,
    endSecondDeriv: Double,
) {
    // TODO: this needs a test
    val a = -6.0 * start - 3.0 * startDeriv - 0.5 * startSecondDeriv +
        6.0 * end - 3.0 * endDeriv + 0.5 * endSecondDeriv
    val b = 15.0 * start + 8.0 * startDeriv + 1.5 * startSecondDeriv -
        15.0 * end + 7.0 * endDeriv - endSecondDeriv
    val c = -10.0 * start - 6.0 * startDeriv - 1.5 * startSecondDeriv +
        10.0 * end - 4.0 * endDeriv + 0.5 * endSecondDeriv
    val d = 0.5 * startSecondDeriv
    val e = startDeriv
    val f = start

    // TODO: this needs a test
    operator fun get(t: Double, n: Int) = DualNum<Internal>(DoubleArray(n) {
        when (it) {
            0 -> ((((a * t + b) * t + c) * t + d) * t + e) * t + f
            1 -> (((5.0 * a * t + 4.0 * b) * t + 3.0 * c) * t + 2.0 * d) * t + e
            2 -> ((20.0 * a * t + 12.0 * b) * t + 6.0 * c) * t + 2.0 * d
            3 -> (60.0 * a * t + 24.0 * b) * t + 6.0 * c
            4 -> 120.0 * a * t + 24.0 * b
            5 -> 120.0 * a
            else -> 0.0
        }
    })
}

interface PositionPath<Param> {
    val maxParam: Double
    operator fun get(param: Double): Position2<DualNum<Param>>
}

//class

data class ScanResult(
    val values: List<Double>,
    val sums: List<Double>,
)

// implementation of adaptsim from Gander and Gautschi
// TODO: test this
fun integralScan(f: (Double) -> Double, a: Double, b: Double, eps: Double): ScanResult {
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

object ArcLength

class ArcApproxArcCurve2(
    val curve: PositionPath<Internal>,
) {
//    : PositionPath<ArcLength> {
    val samples = integralScan({
        sqrt(
            curve[it].x.values[1] * curve[it].x.values[1] +
            curve[it].y.values[1] * curve[it].y.values[1]
        )
    }, 0.0, curve.maxParam, 1e-6)

    init {
        println(samples.sums.last())
    }
}

fun main() {
//    ArcApproxArcCurve2()
}
