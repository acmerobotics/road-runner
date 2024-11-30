package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import org.knowm.xchart.QuickChart
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.theme.MatlabTheme
import kotlin.math.pow
import kotlin.random.Random
import kotlin.test.assertEquals

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
    val xs = ts.map { p[it] }

    val chart = QuickChart.getChart(
        "Profile",
        "t",
        "",
        arrayOf("x", "v", "a"),
        ts.toDoubleArray(),
        arrayOf(
            xs.map { it[0] }.toDoubleArray(),
            xs.map { it[1] }.toDoubleArray(),
            xs.map { it[2] }.toDoubleArray(),
        ),
    )

    chart.styler.theme = MatlabTheme()

    return chart
}

fun saveProfiles(s: String, p: TimeProfile) {
    saveChart("profile/${s}Disp", chartDispProfile(p.dispProfile))
    saveChart("profile/${s}Time", chartTimeProfile(p))
}

class ProfilesTest {
    @Test
    fun testBeginAccel() {
        val p = TimeProfile(constantProfile(10.0, 0.0, 5.0, -5.0, 5.0).baseProfile)
        assertEquals(5.0, p.dispProfile[0.0][2], 1e-6)
        assertEquals(5.0, p[0.0][2], 1e-6)
    }

    @Test
    fun testSimpleProfile() =
        saveProfiles(
            "simpleProfile",
            TimeProfile(
                constantProfile(10.0, 0.0, 5.0, -5.0, 5.0).baseProfile
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
                ).baseProfile
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
                ).baseProfile
            )
        )

    @Test
    fun testProfileComplexNonZeroMinCancel() =
        saveProfiles(
            "profileNonZeroCancel",
            TimeProfile(
                profile(
                    10.0,
                    3.0,
                    { (it - 5.0).pow(4.0) + 1.0 },
                    { -5.0 },
                    { 5.0 },
                    0.01,
                ).cancel(2.0)
            )
        )

    @Test
    fun testProfileComplexNonZeroMinCancel2() {
        val profile = profile(
            10.0,
            3.0,
            { (it - 5.0).pow(4.0) + 1.0 },
            { -5.0 },
            { 5.0 },
            0.01,
        ).cancel(-2.0)

        saveProfiles(
            "profileNonZeroCancel2",
            TimeProfile(profile)
        )
    }

    @Test
    fun testSimpleAsymmetricProfile() =
        saveProfiles(
            "asymmetricProfile",
            TimeProfile(
                constantProfile(10.0, 0.0, 5.0, -2.0, 5.0).baseProfile
            )
        )

    @Test
    fun testTimeProfileInverse() {
        val profile = TimeProfile(
            profile(
                10.0,
                3.0,
                { (it - 5.0).pow(4.0) + 1.0 },
                { -5.0 },
                { 5.0 },
                0.01,
            ).baseProfile
        )

        repeat(100) {
            val t = Random.Default.nextDouble(profile.duration + 2.0) - 1.0
            assertEquals(clamp(t, 0.0, profile.duration), profile.inverse(profile[t][0]), 1e-2)
        }
    }

    @Test
    fun testIssue103CancelZero() {
        val p = profile(
            10.0,
            3.0,
            { (it - 5.0).pow(4.0) + 1.0 },
            { -5.0 },
            { 5.0 },
            0.01,
        )
        assertEquals(DisplacementProfile(listOf(0.0), listOf(3.0), emptyList()), p.cancel(0.0))
        assertEquals(DisplacementProfile(listOf(0.0), listOf(3.0), emptyList()), p.cancel(-1.0))
    }

    @Test
    fun testSemicircleProfile() =
        saveProfiles(
            "semicircleProfile",
            TimeProfile(
                TrajectoryBuilder(
                    TEST_TRAJECTORY_BUILDER_PARAMS,
                    Pose2d(0.0, 0.0, 0.0),
                    0.0,
                    MinVelConstraint(
                        listOf(
                            MecanumKinematics(10.0).WheelVelConstraint(10.0),
                            AngularVelConstraint(10.0)
                        )
                    ),
                    ProfileAccelConstraint(-10.0, 10.0),
                )
                    .splineTo(Vector2d(30.0, 30.0), Math.PI / 2)
                    .splineTo(Vector2d(0.0, 60.0), Math.PI)
                    .build()
                    .first()
                    .profile.baseProfile
            )
        )

    @Test
    fun testSemicircle2Profile() =
        saveProfiles(
            "semicircle2Profile",
            TimeProfile(
                TrajectoryBuilder(
                    TEST_TRAJECTORY_BUILDER_PARAMS,
                    Pose2d(0.0, 0.0, 0.0),
                    0.0,
                    TranslationalVelConstraint(10.0),
                    ProfileAccelConstraint(-10.0, 10.0),
                )
                    .splineTo(Vector2d(30.0, 30.0), Math.PI / 2)
                    .splineTo(Vector2d(0.0, 60.0), Math.PI)
                    .build()
                    .first()
                    .profile.baseProfile
            )
        )
}
