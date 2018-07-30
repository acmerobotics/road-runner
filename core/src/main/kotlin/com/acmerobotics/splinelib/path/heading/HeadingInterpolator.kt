package com.acmerobotics.splinelib.path.heading

import com.acmerobotics.splinelib.path.ParametricCurve

interface HeadingInterpolator {
    fun init(parametricCurve: ParametricCurve)
    fun respectsDerivativeContinuity(): Boolean
    operator fun get(displacement: Double): Double
    fun deriv(displacement: Double): Double
    fun secondDeriv(displacement: Double): Double
}