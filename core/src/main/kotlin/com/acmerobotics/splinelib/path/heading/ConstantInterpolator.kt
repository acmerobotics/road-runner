package com.acmerobotics.splinelib.path.heading

import com.acmerobotics.splinelib.path.ParametricCurve

/**
 * Constant heading interpolator used for arbitrary holonomic translations.
 */
class ConstantInterpolator(val heading: Double) : HeadingInterpolator {
    override fun init(parametricCurve: ParametricCurve) {

    }

    override fun respectsDerivativeContinuity() = false

    override fun get(displacement: Double) = heading

    override fun deriv(displacement: Double) = 0.0

    override fun secondDeriv(displacement: Double) = 0.0

}