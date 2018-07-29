package com.acmerobotics.splinelib.path

interface HeadingInterpolator {
    fun init(parametricCurve: ParametricCurve)
    fun respectsDerivativeContinuity(): Boolean
    operator fun get(displacement: Double): Double
    fun deriv(displacement: Double): Double
    fun secondDeriv(displacement: Double): Double
}