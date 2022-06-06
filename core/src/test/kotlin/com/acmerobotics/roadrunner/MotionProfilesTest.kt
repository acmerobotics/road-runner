package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import org.knowm.xchart.QuickChart
import org.knowm.xchart.style.theme.MatlabTheme
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.test.assertEquals

fun isSorted(xs: List<Double>) =
    xs
        .zip(xs.drop(1))
        .all { (a, b) -> a <= b }

fun double(xs: List<Double>) = List(2 * xs.size) { xs[it / 2] }
fun doubleInner(xs: List<Double>) = listOf(xs.first()) +
        double(xs.slice(1 until xs.lastIndex)) +
        listOf(xs.last())

fun chartProfile(p: MotionProfile) = QuickChart.getChart(
        "Profile",
        "x",
        "",
        arrayOf("v", "a"),
        doubleInner(p.disps).toDoubleArray(),
        arrayOf(
            doubleInner(p.vels).toDoubleArray(),
            double(p.accels).toDoubleArray(),
        )
    ).also {
        it.styler.theme = MatlabTheme()
    }

class MotionProfilesTest {
    @Test
    fun testSimpleProfile() {
        saveChart("simpleProfile", chartProfile(
            profile(10.0, 0.0, { 5.0 }, { Interval(-5.0, 5.0) }, 10.0)
        ))
//
//        val f = forwardProfile(10.0, 0.0, { 5.0 }, { 5.0 }, 5.0)
//        val b = backwardProfile(10.0, { 5.0 }, 0.0, { -5.0 }, 5.0)
//        println(f)
//        println(b)
//        println(merge(f, b))
//        println(merge(b, f))
    }

    @Test
    fun testForwardProfileComplex() {
        val p =
            forwardProfile(
                10.0, 0.0,
                { (it - 5.0).pow(4.0) + 1.0 },
                { 5.0 },
                0.1
            )

        assert(isSorted(p.disps))

        saveChart("forwardProfile", chartProfile(p))
    }

    @Test
    fun testBackwardProfileComplex() {
        val p =
            backwardProfile(
                10.0,
                { (it - 5.0).pow(4.0) + 1.0 },
                0.0,
                { -5.0 },
                0.1
            )

        println(p)

//        assert(isSorted(p.disps))

        saveChart("backwardProfile", chartProfile(p))
    }

    @Test
    fun testProfileComplex() {
        val p =
            profile(
                10.0,
                0.0,
                { (it - 5.0).pow(4.0) + 1.0 },
                { Interval(-5.0, 5.0) },
                0.01,
            )

        println(p.disps.zip(p.vels))

//        assert(isSorted(p.disps))

        saveChart("profile", chartProfile(p))
    }
}
