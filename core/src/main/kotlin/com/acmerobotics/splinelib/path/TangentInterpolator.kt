package com.acmerobotics.splinelib.path

class TangentInterpolator: HeadingInterpolator {
    private lateinit var parametricCurve: ParametricCurve

    override fun init(parametricCurve: ParametricCurve) {
        this.parametricCurve = parametricCurve
    }

    override fun get(displacement: Double) = parametricCurve.tangentAngle(displacement)

    override fun deriv(displacement: Double) = parametricCurve.tangentAngleDeriv(displacement)

    override fun secondDeriv(displacement: Double) = parametricCurve.tangentAngleSecondDeriv(displacement)
}