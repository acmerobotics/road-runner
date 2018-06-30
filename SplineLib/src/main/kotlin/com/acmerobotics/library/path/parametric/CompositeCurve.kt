package com.acmerobotics.library.path.parametric

import com.acmerobotics.library.Vector2d

class CompositeCurve(val segments: List<ParametricCurve>) : ParametricCurve() {
    override fun length() = segments.sumByDouble { it.length() }

    override operator fun get(displacement: Double): Vector2d {
        var remainingDisplacement = displacement
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment[remainingDisplacement]
            }
            remainingDisplacement -= segment.length()
        }
        return segments.last().end()
    }

    override fun deriv(displacement: Double): Vector2d {
        var remainingDisplacement = displacement
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment.deriv(remainingDisplacement)
            }
            remainingDisplacement -= segment.length()
        }
        return segments.last().endDeriv()
    }

    override fun secondDeriv(displacement: Double): Vector2d {
        var remainingDisplacement = displacement
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment.secondDeriv(remainingDisplacement)
            }
            remainingDisplacement -= segment.length()
        }
        return segments.last().endSecondDeriv()
    }

    override fun thirdDeriv(displacement: Double): Vector2d {
        var remainingDisplacement = displacement
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment.thirdDeriv(remainingDisplacement)
            }
            remainingDisplacement -= segment.length()
        }
        return segments.last().endThirdDeriv()
    }

}