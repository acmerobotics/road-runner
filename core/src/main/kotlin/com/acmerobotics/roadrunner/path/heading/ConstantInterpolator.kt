package com.acmerobotics.roadrunner.path.heading

/**
 * Constant heading interpolator used for arbitrary holonomic translations.
 *
 * @param heading heading to maintain
 */
class ConstantInterpolator(val heading: Double) : HeadingInterpolator() {
    override fun respectsDerivativeContinuity() = false

    override fun get(s: Double) = heading

    override fun deriv(s: Double) = 0.0

    override fun secondDeriv(s: Double) = 0.0

}