package com.acmerobotics.roadrunner.path.heading

import com.acmerobotics.roadrunner.path.ParametricCurve
import com.acmerobotics.roadrunner.path.QuinticPolynomial

/**
 * Spline heading interpolator for transitioning smoothly between headings without violating continuity (and hence
 * allowing for integration into longer profiles).
 *
 * @param startHeading start heading
 * @param endHeading end heading
 */
// note: the spline parameter is transformed linearly into a pseudo-arclength parameter
class SplineInterpolator(private val startHeading: Double, private val endHeading: Double) : HeadingInterpolator() {
    private val tangentInterpolator = TangentInterpolator()
    private lateinit var headingSpline: QuinticPolynomial

    override fun init(curve: ParametricCurve) {
        super.init(curve)

        tangentInterpolator.init(this.curve)

        val len = curve.length()

        headingSpline = QuinticPolynomial(
                startHeading,
                curve.tangentAngleDeriv(0.0, 0.0) * len,
                curve.tangentAngleSecondDeriv(0.0, 0.0) * len * len,
                endHeading,
                curve.tangentAngleDeriv(len, 1.0) * len,
                curve.tangentAngleSecondDeriv(len, 1.0) * len * len
        )
    }

    override fun internalGet(s: Double, t: Double) = headingSpline[s / curve.length()]

    override fun internalDeriv(s: Double, t: Double): Double {
        val len = curve.length()
        return headingSpline.deriv(s / len) / len
    }

    override fun internalSecondDeriv(s: Double, t: Double): Double {
        val len = curve.length()
        return headingSpline.secondDeriv(s / len) / (len * len)
    }
}
