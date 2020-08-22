package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.DoubleProgression
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.abs
import kotlin.math.asin

/**
 * Parameterizes a [ParametricCurve] for arc-length (s) based on a recursive arc-based segment subdivision method.
 *
 * @param maxDeltaK maximum change in curvature between arc length param segments
 * @param maxSegmentLength maximum length of a single param segment
 * @param maxDepth maximum stack depth
 */
class SplineParameterizer(
    private val maxDeltaK: Double = 0.01,
    private val maxSegmentLength: Double = 0.25,
    private val maxDepth: Int = 30
) {
    var length = 0.0
        private set

    private lateinit var curve: ParametricCurve

    private val sSamples = mutableListOf(0.0)
    private val tSamples = mutableListOf(0.0)

    /**
     * Performs the parameterize operation on the curve to initialize the internal state
     *
     * @param curve the curve to be parameterized
     * @param tLo starting curve parameter (default 0.0)
     * @param tHi ending curve parameter (default 1.0)
     */
    fun init(curve: ParametricCurve, tLo: Double = 0.0, tHi: Double = 1.0) {
        this.curve = curve
        parameterize(tLo, tHi)
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
        val deriv = curve.internalDeriv(t)
        val derivNorm = deriv.norm()
        val secondDeriv = curve.internalSecondDeriv(t)
        return abs(secondDeriv.x * deriv.y - deriv.x * secondDeriv.y) / (derivNorm * derivNorm * derivNorm)
    }

    private fun parameterize(
        tLo: Double,
        tHi: Double,
        vLo: Vector2d = curve.internalGet(tLo),
        vHi: Vector2d = curve.internalGet(tHi),
        depth: Int = 0
    ) {
        val tMid = 0.5 * (tLo + tHi)
        val vMid = curve.internalGet(tMid)

        val deltaK = abs(internalCurvature(tLo) - internalCurvature(tHi))
        val segmentLength = approxLength(vLo, vMid, vHi)

        if ((deltaK > maxDeltaK || segmentLength > maxSegmentLength) && depth < maxDepth) {
            parameterize(tLo, tMid, vLo, vMid, depth + 1)
            parameterize(tMid, tHi, vMid, vHi, depth + 1)
        } else {
            length += segmentLength
            sSamples.add(length)
            tSamples.add(tHi)
        }
    }

    private fun interp(s: Double, sLo: Double, sHi: Double, tLo: Double, tHi: Double) =
            tLo + (s - sLo) * (tHi - tLo) / (sHi - sLo)

    /**
     * Reparameterizes the arc-length parameter [s] into the curve's t parameter
     *
     * @param s the displacement/arc-length param
     */
    fun reparam(s: Double): Double {
        if (s <= 0.0) return 0.0
        if (s >= length) return 1.0

        var lo = 0
        var hi = sSamples.size

        while (lo <= hi) {
            val mid = (hi + lo) / 2

            when {
                s < sSamples[mid] -> {
                    hi = mid - 1
                }
                s > sSamples[mid] -> {
                    lo = mid + 1
                }
                else -> {
                    return tSamples[mid]
                }
            }
        }

        return interp(s, sSamples[lo], sSamples[hi], tSamples[lo], tSamples[hi])
    }

    /**
     * Reparameterizes [s] into the curve's t parameter
     *
     * @param s the displacement/arc-length param progression
     */
    fun reparam(s: DoubleProgression): DoubleArray {
        val t = DoubleArray(s.size())
        var i = 0
        var sampleIndex = 0
        var currS = s.start
        while (i < t.size) {
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
}
