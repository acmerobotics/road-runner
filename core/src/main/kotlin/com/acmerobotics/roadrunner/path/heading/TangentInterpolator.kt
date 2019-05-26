package com.acmerobotics.roadrunner.path.heading

/**
 * Tangent (system) interpolator for tank/differential and other nonholonomic drives.
 */
class TangentInterpolator : HeadingInterpolator() {
    override fun respectsDerivativeContinuity() = true

    override fun internalGet(s: Double, t: Double) = curve.tangentAngle(s, t)

    override fun internalDeriv(s: Double, t: Double) = curve.tangentAngleDeriv(s, t)

    override fun internalSecondDeriv(s: Double, t: Double) = curve.tangentAngleSecondDeriv(s, t)
}
