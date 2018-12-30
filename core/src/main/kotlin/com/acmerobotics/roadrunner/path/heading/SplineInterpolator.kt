package com.acmerobotics.roadrunner.path.heading

import com.acmerobotics.roadrunner.path.ParametricCurve
import com.acmerobotics.roadrunner.path.QuinticPolynomial

/**
 * Spline heading interpolator for transitioning smoothly between headings without violating continuity (and hence
 * allowing for integration into longer profiles).
 */
class SplineInterpolator(private val startHeading: Double, private val endHeading: Double) : HeadingInterpolator() {
    private val tangentInterpolator = TangentInterpolator()
    private lateinit var headingSpline: QuinticPolynomial

    override fun init(parametricCurve: ParametricCurve) {
        super.init(parametricCurve)

        tangentInterpolator.init(this.parametricCurve)

        val len = parametricCurve.length()

        headingSpline = QuinticPolynomial(
                startHeading,
                parametricCurve.tangentAngleDeriv(0.0) * len,
                parametricCurve.tangentAngleSecondDeriv(0.0) * len * len,
                endHeading,
                parametricCurve.tangentAngleDeriv(len) * len,
                parametricCurve.tangentAngleSecondDeriv(len) * len * len
        )
    }

    override fun respectsDerivativeContinuity() = true

    override operator fun get(displacement: Double) = headingSpline[displacement / parametricCurve.length()]

    override fun deriv(displacement: Double): Double {
        val len = parametricCurve.length()
        return headingSpline.deriv(displacement / len) / len
    }

    override fun secondDeriv(displacement: Double): Double {
        val len = parametricCurve.length()
        return headingSpline.secondDeriv(displacement / len) / (len * len)
    }

}