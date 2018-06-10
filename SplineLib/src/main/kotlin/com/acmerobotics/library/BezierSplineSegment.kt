package com.acmerobotics.library

import kotlin.math.abs

class BezierSplineSegment(start: Waypoint, end: Waypoint) {
    private val p0: Vector2d = start.pos()
    private val p1: Vector2d
    private val p2: Vector2d
    private val p3: Vector2d
    private val p4: Vector2d
    private val p5: Vector2d = end.pos()

    init {
        p1 = 0.2 * start.deriv() + p0
        p2 = 0.05 * start.secondDeriv() + 2.0 * p1 - p0

        p4 = p5 - 0.2 * end.deriv()
        p3 = 0.05 * end.secondDeriv() + 2.0 * p4 - p5
    }

    operator fun get(t: Double): Vector2d {
        val s = 1.0 - t
        return (s * p0 + 5.0 * t * p1) * s * s * s * s +
                10.0 * (s * p2 + t * p3) * s * s * t * t +
                (5.0 * s * p4 + t * p5) * t * t * t * t
    }

    fun deriv(t: Double): Vector2d {
        val s = 1.0 - t
        return 5.0 * ((s * (p1 - p0) + 4.0 * t * (p2 - p1)) * s * s * s +
                (6.0 * s * (p3 - p2) + 4.0 * t * (p4 - p3)) * s * t * t +
                t * t * t * t * (p5 - p4))
    }

    fun secondDeriv(t: Double): Vector2d {
        val s = 1.0 - t
        return 20.0 * ((s * (p2 - 2.0 * p1 + p0) + 3.0 * t * (p3 - 2.0 * p2 + p1)) * s * s +
                (3.0 * s * (p4 - 2.0 * p3 + p2) + t * (p5 - 2.0 * p4 + p3)) * t * t)
    }

    fun curvature(t: Double): Double {
        val deriv = deriv(t)
        val secondDeriv = secondDeriv(t)
        val norm = deriv.norm()
        return abs(deriv.x * secondDeriv.y - deriv.y * secondDeriv.x) / (norm * norm * norm)
    }
}