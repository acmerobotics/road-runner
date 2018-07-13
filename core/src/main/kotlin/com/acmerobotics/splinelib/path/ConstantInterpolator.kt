package com.acmerobotics.splinelib.path

class ConstantInterpolator(val heading: Double) : HeadingInterpolator {
    override fun init(parametricCurve: ParametricCurve) {

    }

    override fun get(displacement: Double) = heading

    override fun deriv(displacement: Double) = 0.0

    override fun secondDeriv(displacement: Double) = 0.0

}