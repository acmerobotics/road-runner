package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import org.knowm.xchart.QuickChart
import org.knowm.xchart.style.theme.MatlabTheme
import kotlin.math.pow

fun isSorted(xs: List<Double>) =
    xs
        .zip(xs.drop(1))
        .all { (a, b) -> a <= b }

fun double(xs: List<Double>) = List(2 * xs.size) { xs[it / 2] }
fun doubleInner(xs: List<Double>) = listOf(xs.first()) +
        double(xs.slice(1 until xs.lastIndex)) +
        listOf(xs.last())

// TODO: change this to sampling
// should fix the broken appearance of constant limit profiles
// extend the displacement range in both directions
fun chartProfile(p: DisplacementProfile) = QuickChart.getChart(
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
            profile(10.0, 0.0, { 5.0 }, { Interval(-5.0, 5.0) }, 0.1)
        ))
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

//        assert(isSorted(p.disps))

        saveChart("profile", chartProfile(p))
    }

    @Test
    fun testProfileComplexNonZeroMin() {
        val p =
            profile(
                10.0,
                0.5,
                { (it - 5.0).pow(4.0) + 1.0 },
                { Interval(-5.0, 5.0) },
                0.01,
            )

//        assert(isSorted(p.disps))

        saveChart("profileNonZero", chartProfile(p))
    }

    @Test
    fun testSimpleAsymmetricProfile() {
        saveChart(
            "asymmetricProfile", chartProfile(
                profile(10.0, 0.0, { 5.0 }, { Interval(-2.0, 5.0) }, 10.0)
            )
        )
    }
}
