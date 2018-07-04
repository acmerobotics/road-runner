package com.acmerobotics.library.path.heading

import com.acmerobotics.library.path.parametric.QuinticSplineSegment

class TangentInterpolator: HeadingInterpolator {
    private lateinit var spline: QuinticSplineSegment

    override fun fit(spline: QuinticSplineSegment) {
        this.spline = spline
    }

    override fun get(displacement: Double): Double {
        val pathDeriv = spline.deriv(displacement)
        val angle = Math.atan2(pathDeriv.y, pathDeriv.x)
        return if (angle.isNaN()) 0.0 else angle
    }

    override fun deriv(displacement: Double): Double {
        val pathDeriv = spline.deriv(displacement)
        val pathSecondDeriv = spline.secondDeriv(displacement)

        var deriv = pathDeriv.x * pathSecondDeriv.y - pathSecondDeriv.x * pathDeriv.y
        deriv /= (pathDeriv.x * pathDeriv.x + pathDeriv.y * pathDeriv.y)

        return if (deriv.isNaN()) 0.0 else deriv
    }

    override fun secondDeriv(displacement: Double): Double {
        val pathDeriv = spline.deriv(displacement)
        val pathSecondDeriv = spline.secondDeriv(displacement)
        val pathThirdDeriv = spline.thirdDeriv(displacement)

        // if you're curious and hate yourself enough, here's the complete formula:
        // http://www.wolframalpha.com/input/?i=d%2Fds(d%2Fds(arctan((dy%2Fds)%2F(dx%2Fds))))
        val denominator = pathDeriv.x * pathDeriv.x + pathDeriv.y * pathDeriv.y
        val firstTerm = (pathThirdDeriv.y * pathDeriv.x - pathThirdDeriv.x * pathDeriv.y) / denominator
        var secondTerm = (pathDeriv.x * pathSecondDeriv.y - pathSecondDeriv.x * pathDeriv.y)
        secondTerm *= 2 * (pathDeriv.x * pathSecondDeriv.x + pathDeriv.y * pathSecondDeriv.y)
        secondTerm /= (denominator * denominator)
        val secondDeriv = firstTerm - secondTerm

        return if (secondDeriv.isNaN()) 0.0 else secondDeriv
    }
}