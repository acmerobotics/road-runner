package com.acmerobotics.roadrunner.path.heading

/**
 * Tangent (system) interpolator for tank/differential and other nonholonomic drives.
 */
class TangentInterpolator: HeadingInterpolator() {
    override fun respectsDerivativeContinuity() = true

    override fun get(s: Double) = parametricCurve.tangentAngle(s)

    override fun deriv(s: Double) = parametricCurve.tangentAngleDeriv(s)

    override fun secondDeriv(s: Double) = parametricCurve.tangentAngleSecondDeriv(s)
}