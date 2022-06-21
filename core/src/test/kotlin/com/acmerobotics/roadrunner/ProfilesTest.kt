package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import org.knowm.xchart.QuickChart
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.theme.MatlabTheme
import kotlin.math.pow
import kotlin.test.assertEquals

fun isSorted(xs: List<Double>) =
    xs
        .zip(xs.drop(1))
        .all { (a, b) -> a <= b }

fun double(xs: List<Double>) = List(2 * xs.size) { xs[it / 2] }
fun doubleInner(xs: List<Double>) = listOf(xs.first()) +
    double(xs.slice(1 until xs.lastIndex)) +
    listOf(xs.last())

fun chartDispProfile(p: DisplacementProfile): XYChart {
    val xs = range(-0.1 * p.length, 1.1 * p.length, 1000)

    val chart = QuickChart.getChart(
        "Profile",
        "x",
        "",
        arrayOf("v", "a"),
        xs.toDoubleArray(),
        xs.map {
            val xDual = p[it]
            Pair(xDual[1], xDual[2])
        }.unzip().let {
            arrayOf(it.first.toDoubleArray(), it.second.toDoubleArray())
        },
    )

    chart.styler.theme = MatlabTheme()

    return chart
}

fun chartTimeProfile(p: TimeProfile): XYChart {
    val ts = range(-0.1 * p.duration, 1.1 * p.duration, 1000)

    val chart = QuickChart.getChart(
        "Profile",
        "t",
        "",
        arrayOf("v", "a"),
        ts.toDoubleArray(),
        ts.map {
            val xDual = p[it]
            Pair(xDual[1], xDual[2])
        }.unzip().let {
            arrayOf(it.first.toDoubleArray(), it.second.toDoubleArray())
        },
    )

    chart.styler.theme = MatlabTheme()

    return chart
}

fun saveProfiles(s: String, p: TimeProfile) {
    saveChart(s + "Disp", chartDispProfile(p.dispProfile))
    saveChart(s + "Time", chartTimeProfile(p))
}

class ProfilesTest {
    @Test
    fun testBeginAccel() {
        val p = TimeProfile(constantProfile(10.0, 0.0, 5.0, -5.0, 5.0))
        assertEquals(5.0, p.dispProfile[0.0][2], 1e-6)
        assertEquals(5.0, p[0.0][2], 1e-6)
    }

    @Test
    fun testSimpleProfile() =
        saveProfiles(
            "simpleProfile",
            TimeProfile(
                constantProfile(10.0, 0.0, 5.0, -5.0, 5.0)
            )
        )

    @Test
    fun testForwardProfileComplex() =
        saveProfiles(
            "forwardProfile",
            TimeProfile(
                forwardProfile(
                    10.0, 0.0,
                    { (it - 5.0).pow(4.0) + 1.0 },
                    { 5.0 },
                    0.1
                )
            )
        )

    @Test
    fun testBackwardProfileComplex() =
        saveProfiles(
            "backwardProfile",
            TimeProfile(
                backwardProfile(
                    10.0,
                    { (it - 5.0).pow(4.0) + 1.0 },
                    0.0,
                    { -5.0 },
                    0.1
                )
            )
        )

    @Test
    fun testProfileComplex() =
        saveProfiles(
            "profile",
            TimeProfile(
                profile(
                    10.0,
                    0.0,
                    { (it - 5.0).pow(4.0) + 1.0 },
                    { -5.0 },
                    { 5.0 },
                    0.01,
                )
            )
        )

    @Test
    fun testProfileComplexNonZeroMin() =
        saveProfiles(
            "profileNonZero",
            TimeProfile(
                profile(
                    10.0,
                    3.0,
                    { (it - 5.0).pow(4.0) + 1.0 },
                    { -5.0 },
                    { 5.0 },
                    0.01,
                )
            )
        )

    @Test
    fun testSimpleAsymmetricProfile() =
        saveProfiles(
            "asymmetricProfile",
            TimeProfile(
                constantProfile(10.0, 0.0, 5.0, -2.0, 5.0)
            )
        )
}
