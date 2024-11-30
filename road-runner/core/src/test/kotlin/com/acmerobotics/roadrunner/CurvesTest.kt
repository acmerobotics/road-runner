package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import org.knowm.xchart.QuickChart
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.theme.MatlabTheme
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.min
import kotlin.math.sqrt
import kotlin.random.Random
import kotlin.test.assertEquals

infix fun Vector2d.det(other: Vector2d) = x * other.y - y * other.x

fun Vector2d.free() = Vector2d(x, y)

private fun approxLength(
    p1: Vector2d,
    p2: Vector2d,
    p3: Vector2d
): Double {
    val chord = (p3 - p1).norm()

    val v1 = p2 - p1
    val v2 = p2 - p3
    val det = 4.0 * (v1 det v2)

    return if (abs(det) < 1e-6) {
        chord
    } else {
        val x1 = p1.free().sqrNorm()
        val x2 = p2.free().sqrNorm()
        val x3 = p3.free().sqrNorm()

        val y1 = x2 - x1
        val y2 = x2 - x3

        val center = Vector2d(
            (y1 * v2.y - y2 * v1.y) / det, (y2 * v1.x - y1 * v2.x) / det
        )
        val radius = (p1 - center).norm()
        2.0 * radius * asin(chord / (2.0 * radius))
    }
}

fun Vector2dDual<Internal>.curvature(): Double {
    val (_, dx, d2x) = x.values()
    val (_, dy, d2y) = y.values()
    val derivNorm = sqrt(dx * dx + dy * dy)
    return abs(d2x * dy - dx * d2y) / (derivNorm * derivNorm * derivNorm)
}

class ArcApproxArcCurve2(
    private val curve: PositionPath<Internal>,
    private val maxDeltaK: Double = 0.01,
    private val maxSegmentLength: Double = 0.25,
    private val maxDepth: Int = 30,
) {
    private data class Samples(
        val length: Double,
        // first: s, second: t
        val values: List<Pair<Double, Double>>,
    ) {
        init {
            if (values.size < 2) {
                throw IllegalArgumentException("must have at least two samples")
            }
        }

        operator fun plus(other: Samples) = Samples(
            length + other.length,
            values.dropLast(1) + other.values
        )
    }

    private fun adaptiveSample(): Samples {
        fun helper(
            sLo: Double,
            tLo: Double,
            tHi: Double,
            pLo: Vector2dDual<Internal>,
            pHi: Vector2dDual<Internal>,
            depth: Int,
        ): Samples {
            val tMid = 0.5 * (tLo + tHi)
            val pMid = curve[tMid, 3]

            val deltaK = abs(pLo.curvature() - pHi.curvature())
            val length = approxLength(pLo.value(), pMid.value(), pHi.value())

            return if (depth < maxDepth && (deltaK > maxDeltaK || length > maxSegmentLength)) {
                val loSamples = helper(sLo, tLo, tMid, pLo, pMid, depth + 1)
                // loSamples.length is more accurate than length
                val sMid = sLo + loSamples.length
                val hiSamples = helper(sMid, tMid, tHi, pMid, pHi, depth + 1)
                loSamples + hiSamples
            } else {
                Samples(
                    length,
                    listOf(
                        Pair(sLo, tLo),
                        Pair(sLo + length, tHi)
                    )
                )
            }
        }

        return helper(0.0, 0.0, 1.0, curve[0.0, 3], curve[1.0, 3], 0)
    }

    private val samples: List<Pair<Double, Double>>
    val length: Double

    init {
        val (length, values) = adaptiveSample()
        this.length = length
        this.samples = values
    }

    fun reparam(s: Double): Double {
        val result = samples.binarySearch { (sMid, _) -> sMid.compareTo(s) }
        return when {
            result >= 0 -> samples[result].let { (_, t) -> t }
            else -> {
                val insIndex = -(result + 1)
                when {
                    insIndex == 0 -> 0.0
                    insIndex >= samples.size -> 1.0
                    else -> {
                        val (sLo, tLo) = samples[insIndex - 1]
                        val (sHi, tHi) = samples[insIndex]
                        lerp(s, sLo, sHi, tLo, tHi)
                    }
                }
            }
        }
    }
}

fun numericalDerivative(x: List<Double>, ds: Double): List<Double> {
    val deriv = (0 until x.size - 2).map {
        (x[it + 2] - x[it]) / (2 * ds)
    }.toMutableList()
    deriv.add(0, deriv.first())
    deriv.add(deriv.last())
    return deriv
}

fun chartSplineHeadingPath(p: SplineHeadingPath): XYChart {
    val ss = range(0.0, p.length, 1000)
    val rs = ss.map { p[it, 3] }

    return QuickChart.getChart(
        "Spline Heading Path", "s", "",
        arrayOf("r", "dr", "d2r"),
//        , "dr num", "d2r num"),
//        , "i", "di", "d2i"),
        ss.toDoubleArray(),
        arrayOf(
//            rs.map { it.real[0] }.toDoubleArray(),
//            rs.map { it.real[1] }.toDoubleArray(),
//            rs.map { it.real[2] }.toDoubleArray(),
//            numericalDerivative(rs.map { it.real[0] }, p.length / 999).toDoubleArray(),
//            numericalDerivative(rs.map { it.real[1] }, p.length / 999).toDoubleArray(),
//            rs.map { it.imag[0] }.toDoubleArray(),
//            rs.map { it.imag[1] }.toDoubleArray(),
//            rs.map { it.imag[2] }.toDoubleArray(),

            rs.map { it.value().log() }.toDoubleArray(),
            rs.map { it.velocity()[0] }.toDoubleArray(),
            rs.map { it.velocity()[1] }.toDoubleArray(),
//            numericalDerivative(rs.map { it.log()[0] }, p.length / 999).toDoubleArray(),
//            numericalDerivative(rs.map { it.log()[1] }, p.length / 999).toDoubleArray(),
        )
    ).also {
        it.styler.theme = MatlabTheme()
    }
}

class CurvesTest {
    @Test
    fun testArcLengthReparam() {
        val spline =
            QuinticSpline2d(
                QuinticSpline1d(
                    DualNum(doubleArrayOf(0.0, 10.0, 30.0)),
                    DualNum(doubleArrayOf(20.0, 30.0, 0.0)),
                ),
                QuinticSpline1d(
                    DualNum(doubleArrayOf(0.0, 15.0, 10.0)),
                    DualNum(doubleArrayOf(20.0, 20.0, 0.0)),
                ),
            )

        val curveExp = ArcApproxArcCurve2(spline)
        val curveActual = ArclengthReparamCurve2d(spline, 1e-6)

        range(0.0, min(curveExp.length, curveActual.length), 100)
            .forEach {
                assertEquals(curveExp.reparam(it), curveActual.reparam(it), 1e-2)
            }
    }

    @Test
    fun testSplineInterpolation() {
        val r = Random.Default
        repeat(100) {
            val begin = DualNum<Internal>(doubleArrayOf(r.nextDouble(), r.nextDouble(), r.nextDouble()))
            val end = DualNum<Internal>(doubleArrayOf(r.nextDouble(), r.nextDouble(), r.nextDouble()))

            val spline = QuinticSpline1d(begin, end)

            val splineBegin = spline[0.0, 3]
            assertEquals(begin[0], splineBegin[0], 1e-6)
            assertEquals(begin[1], splineBegin[1], 1e-6)
            assertEquals(begin[2], splineBegin[2], 1e-6)

            val splineEnd = spline[1.0, 3]
            assertEquals(end[0], splineEnd[0], 1e-6)
            assertEquals(end[1], splineEnd[1], 1e-6)
            assertEquals(end[2], splineEnd[2], 1e-6)
        }
    }

    @Test
    fun testSplineGet() {
        val r = Random.Default
        repeat(100) {
            val spline = QuinticSpline1d(
                DualNum(doubleArrayOf(r.nextDouble(), r.nextDouble(), r.nextDouble())),
                DualNum(doubleArrayOf(r.nextDouble(), r.nextDouble(), r.nextDouble())),
            )

            val t = DualNum.variable<Internal>(r.nextDouble(1.0), 4)
            spline[t.value(), 4].values()
                .zip(
                    (
                        t * (
                            t * (
                                t * (
                                    t * (t * spline.a + spline.b) +
                                        spline.c
                                    ) + spline.d
                                ) + spline.e
                            ) + spline.f
                        ).values()
                )
                .forEach {
                    assertEquals(it.first, it.second, 1e-6)
                }
        }
    }

    @Test
    fun testSplineHeadingInterpolation() {
        val r = Random.Default
        repeat(100) {
            val begin = Rotation2dDual.exp(
                DualNum<Arclength>(doubleArrayOf(r.nextDouble(), r.nextDouble(), r.nextDouble()))
            )
            val end = Rotation2dDual.exp(
                DualNum<Arclength>(doubleArrayOf(r.nextDouble(), r.nextDouble(), r.nextDouble()))
            )

            val spline = SplineHeadingPath(begin, end, abs(r.nextDouble()))

            val splineBegin = spline.begin(3)
            assertEquals(begin.real[0], splineBegin.real[0], 1e-6)
            assertEquals(begin.real[1], splineBegin.real[1], 1e-6)
            assertEquals(begin.real[2], splineBegin.real[2], 1e-6)

            assertEquals(begin.imag[0], splineBegin.imag[0], 1e-6)
            assertEquals(begin.imag[1], splineBegin.imag[1], 1e-6)
            assertEquals(begin.imag[2], splineBegin.imag[2], 1e-6)

            assertEquals(begin.velocity()[0], splineBegin.velocity()[0], 1e-6)
            assertEquals(begin.velocity()[1], splineBegin.velocity()[1], 1e-6)

            assertEquals(begin.velocity()[0], splineBegin.velocity()[0], 1e-6)
            assertEquals(begin.velocity()[1], splineBegin.velocity()[1], 1e-6)

            val splineEnd = spline.end(3)
            assertEquals(end.real[0], splineEnd.real[0], 1e-6)
            assertEquals(end.real[1], splineEnd.real[1], 1e-6)
            assertEquals(end.real[2], splineEnd.real[2], 1e-6)

            assertEquals(end.imag[0], splineEnd.imag[0], 1e-6)
            assertEquals(end.imag[1], splineEnd.imag[1], 1e-6)
            assertEquals(end.imag[2], splineEnd.imag[2], 1e-6)

            assertEquals(end.velocity()[0], splineEnd.velocity()[0], 1e-6)
            assertEquals(end.velocity()[1], splineEnd.velocity()[1], 1e-6)

            assertEquals(end.velocity()[0], splineEnd.velocity()[0], 1e-6)
            assertEquals(end.velocity()[1], splineEnd.velocity()[1], 1e-6)
        }
    }

    @Test
    fun testSplineHeadingPath() {
        val p = SplineHeadingPath(
            Rotation2dDual.exp(DualNum(doubleArrayOf(PI / 2, 1.0, -1.0))),
            Rotation2dDual.exp(DualNum(doubleArrayOf(-PI / 6, -0.5, 1.5))),
            15.0,
        )

        saveChart("splineHeadingPathSpline", chartSpline(p.spline))
        saveChart("splineHeadingPath", chartSplineHeadingPath(p))
    }
}
