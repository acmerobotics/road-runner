package com.acmerobotics.library

import kotlin.math.sqrt

class BezierSplineSegment(
    private val p0: Vector2d,
    private val p1: Vector2d,
    private val p2: Vector2d,
    private val p3: Vector2d,
    private val p4: Vector2d,
    private val p5: Vector2d
) : Path() {
    companion object {
        private const val LENGTH_SAMPLES = 1000

        fun fromWaypoints(start: Waypoint, end: Waypoint): BezierSplineSegment {
            val p0 = start.pos()
            val p5 = end.pos()

            val p1 = 0.2 * start.deriv() + p0
            val p2 = 0.05 * start.secondDeriv() + 2.0 * p1 - p0

            val p4 = p5 - 0.2 * end.deriv()
            val p3 = 0.05 * end.secondDeriv() + 2.0 * p4 - p5

            return BezierSplineSegment(p0, p1, p2, p3, p4, p5)
        }
    }

    private val length by lazy { computeLength(LENGTH_SAMPLES) }

    override fun length() = length

    override operator fun get(displacement: Double): Vector2d {
        val t = displacement / length
        val s = 1.0 - t
        return (s * p0 + 5.0 * t * p1) * s * s * s * s +
                10.0 * (s * p2 + t * p3) * s * s * t * t +
                (5.0 * s * p4 + t * p5) * t * t * t * t
    }

    override fun deriv(displacement: Double): Vector2d {
        val t = displacement / length
        val s = 1.0 - t
        return 5.0 * ((s * (p1 - p0) + 4.0 * t * (p2 - p1)) * s * s * s +
                (6.0 * s * (p3 - p2) + 4.0 * t * (p4 - p3)) * s * t * t +
                t * t * t * t * (p5 - p4))
    }

    override fun secondDeriv(displacement: Double): Vector2d {
        val t = displacement / length
        val s = 1.0 - t
        return 20.0 * ((s * (p2 - 2.0 * p1 + p0) + 3.0 * t * (p3 - 2.0 * p2 + p1)) * s * s +
                (3.0 * s * (p4 - 2.0 * p3 + p2) + t * (p5 - 2.0 * p4 + p3)) * t * t)
    }

    override fun thirdDeriv(displacement: Double): Vector2d {
        TODO()
    }

    private fun computeLength(samples: Int): Double {
        val dx = 1.0 / samples
        var sum = 0.0
        var lastIntegrand = 0.0
        for (i in 1..samples) {
            val t = i * dx
            val deriv = deriv(t)
            val integrand = sqrt(deriv.x * deriv.x + deriv.y * deriv.y)
            sum += (integrand + lastIntegrand) / 2.0
            lastIntegrand = integrand
        }
        return sum * dx
    }
}