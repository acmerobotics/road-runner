package com.acmerobotics.splinelib.path.heading

import com.acmerobotics.splinelib.path.ParametricCurve

/**
 * Tangent (default) interpolator for tank/differential and other nonholonomic drives.
 */
class TangentInterpolator: HeadingInterpolator {
    private lateinit var parametricCurve: ParametricCurve

    override fun init(parametricCurve: ParametricCurve) {
        this.parametricCurve = parametricCurve
    }

    override fun respectsDerivativeContinuity() = true

    override fun get(displacement: Double) = parametricCurve.tangentAngle(displacement)

    override fun deriv(displacement: Double) = parametricCurve.tangentAngleDeriv(displacement)

    override fun secondDeriv(displacement: Double) = parametricCurve.tangentAngleSecondDeriv(displacement)
}