package com.acmerobotics.roadrunner.path.heading

/**
 * Constant heading interpolator used for arbitrary holonomic translations.
 *
 * @param heading heading to maintain
 */
class ConstantInterpolator(val heading: Double) : HeadingInterpolator() {
    override fun respectsDerivativeContinuity() = false

    override fun internalGet(s: Double, t: Double): Double = heading

    override fun internalDeriv(s: Double, t: Double) = 0.0

    override fun internalSecondDeriv(s: Double, t: Double) = 0.0
}