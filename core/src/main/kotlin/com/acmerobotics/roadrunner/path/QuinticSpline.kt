package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.DoubleProgression
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.sqrt

/**
 * Combination of two quintic polynomials into a 2D quintic spline. See
 * [this short paper](https://github.com/acmerobotics/road-runner/blob/master/doc/pdf/Quintic_Splines_for_FTC.pdf) for
 * some motivation and implementation details.
 *
 * @param start start waypoint
 * @param end end waypoint
 * @param maxDeltaK maximum change in curvature between arc length param segments
 * @param maxSegmentLength maximum length of a single param segment
 */
class QuinticSpline(
    start: Waypoint,
    end: Waypoint,
    private val maxDeltaK: Double = 0.01,
    private val maxSegmentLength: Double = 0.25
) : ParametricCurve() {

    /**
     * X polynomial (i.e., x(t))
     */
    val x: QuinticPolynomial = QuinticPolynomial(start.x, start.dx, start.d2x, end.x, end.dx, end.d2x)

    /**
     * Y polynomial (i.e., y(t))
     */
    val y: QuinticPolynomial = QuinticPolynomial(start.y, start.dy, start.d2y, end.y, end.dy, end.d2y)

    /**
     * Class for representing the end points of interpolated quintic splines.
     *
     * @param x x position
     * @param y y position
     * @param dx x derivative
     * @param dy y derivative
     * @param d2x x second derivative
     * @param d2y y second derivative
     */
    class Waypoint @JvmOverloads constructor(
        val x: Double,
        val y: Double,
        val dx: Double = 0.0,
        val dy: Double = 0.0,
        val d2x: Double = 0.0,
        val d2y: Double = 0.0
    ) {
        @JvmOverloads constructor(
            pos: Vector2d,
            deriv: Vector2d = Vector2d(),
            secondDeriv: Vector2d = Vector2d()
        ) : this(pos.x, pos.y, deriv.x, deriv.y, secondDeriv.x, secondDeriv.y)

        fun pos() = Vector2d(x, y)

        fun deriv() = Vector2d(dx, dy)

        fun secondDeriv() = Vector2d(d2x, d2y)
    }

    private var length: Double = 0.0

    private val sSamples = mutableListOf(0.0)
    private val tSamples = mutableListOf(0.0)

    init {
        parametrize(0.0, 1.0)
    }

    private fun approxLength(v1: Vector2d, v2: Vector2d, v3: Vector2d): Double {
        val w1 = (v2 - v1) * 2.0
        val w2 = (v2 - v3) * 2.0
        val det = w1.x * w2.y - w2.x * w1.y
        val chord = v1 distTo v3
        return if (det epsilonEquals 0.0) {
            chord
        } else {
            val x1 = v1.x * v1.x + v1.y * v1.y
            val x2 = v2.x * v2.x + v2.y * v2.y
            val x3 = v3.x * v3.x + v3.y * v3.y

            val y1 = x2 - x1
            val y2 = x2 - x3

            val origin = Vector2d(y1 * w2.y - y2 * w1.y, y2 * w1.x - y1 * w2.x) / det
            val radius = origin distTo v1
            2.0 * radius * asin(chord / (2.0 * radius))
        }
    }

    private fun internalCurvature(t: Double): Double {
        val deriv = internalDeriv(t)
        val derivNorm = deriv.norm()
        val secondDeriv = internalSecondDeriv(t)
        return abs(secondDeriv.x * deriv.y - deriv.x * secondDeriv.y) / (derivNorm * derivNorm * derivNorm)
    }

    private fun parametrize(
        tLo: Double,
        tHi: Double,
        vLo: Vector2d = internalGet(tLo),
        vHi: Vector2d = internalGet(tHi)
    ) {
        val tMid = 0.5 * (tLo + tHi)
        val vMid = internalGet(tMid)

        val deltaK = abs(internalCurvature(tLo) - internalCurvature(tHi))
        val segmentLength = approxLength(vLo, vMid, vHi)

        if (deltaK > maxDeltaK || segmentLength > maxSegmentLength) {
            parametrize(tLo, tMid, vLo, vMid)
            parametrize(tMid, tHi, vMid, vHi)
        } else {
            length += segmentLength
            sSamples.add(length)
            tSamples.add(tHi)
        }
    }

    override fun internalGet(t: Double) = Vector2d(x[t], y[t])

    override fun internalDeriv(t: Double) = Vector2d(x.deriv(t), y.deriv(t))

    override fun internalSecondDeriv(t: Double) =
        Vector2d(x.secondDeriv(t), y.secondDeriv(t))

    override fun internalThirdDeriv(t: Double) =
        Vector2d(x.thirdDeriv(t), y.thirdDeriv(t))

    private fun interp(s: Double, sLo: Double, sHi: Double, tLo: Double, tHi: Double) =
        tLo + (s - sLo) * (tHi - tLo) / (sHi - sLo)

    override fun reparam(s: Double): Double {
        if (s <= 0.0) return 0.0
        if (s >= length) return 1.0

        var lo = 0
        var hi = sSamples.size

        while (lo <= hi) {
            val mid = (hi + lo) / 2

            if (s < sSamples[mid]) {
                hi = mid - 1
            } else if (s > sSamples[mid]) {
                lo = mid + 1
            }
        }

        return interp(s, sSamples[lo], sSamples[hi], tSamples[lo], tSamples[hi])
    }

    override fun reparam(s: DoubleProgression): DoubleArray {
        val t = DoubleArray(s.items())
        var i = 0
        var sampleIndex = 0
        var currS = s.start
        while (currS <= s.end) {
            t[i++] = when {
                currS <= 0.0 -> 0.0
                currS >= length -> 1.0
                else -> {
                    while (sSamples[sampleIndex] < currS) {
                        sampleIndex++
                    }
                    val s0 = sSamples[sampleIndex - 1]
                    val s1 = sSamples[sampleIndex]
                    val t0 = tSamples[sampleIndex - 1]
                    val t1 = tSamples[sampleIndex]
                    interp(currS, s0, s1, t0, t1)
                }
            }
            currS += s.step
        }
        while (i < t.size) {
            t[i] = 1.0
            i++
        }
        return t
    }

    override fun paramDeriv(t: Double): Double {
        val deriv = internalDeriv(t)
        return 1.0 / sqrt(deriv.x * deriv.x + deriv.y * deriv.y)
    }

    override fun paramSecondDeriv(t: Double): Double {
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val numerator = -(deriv.x * secondDeriv.x + deriv.y * secondDeriv.y)
        val denominator = deriv.x * deriv.x + deriv.y * deriv.y
        return numerator / (denominator * denominator)
    }

    override fun paramThirdDeriv(t: Double): Double {
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val thirdDeriv = internalThirdDeriv(t)
        val firstNumerator = -(deriv.x * secondDeriv.x + deriv.y * secondDeriv.y)
        val secondNumeratorFirstTerm = secondDeriv.x * secondDeriv.x + secondDeriv.y * secondDeriv.y +
                deriv.x * thirdDeriv.x + deriv.y * thirdDeriv.y
        val secondNumeratorSecondTerm = -4.0 * firstNumerator
        val denominator = deriv.x * deriv.x + deriv.y * deriv.y
        return (secondNumeratorFirstTerm / Math.pow(denominator, 2.5) +
                secondNumeratorSecondTerm / Math.pow(denominator, 3.5))
    }

    override fun length() = length

    override fun toString() = "($x,$y)"
}
