package com.acmerobotics.library.path.heading

import com.acmerobotics.library.path.parametric.ParametricCurve

abstract class HeadingInterpolator(val parametricCurve: ParametricCurve) {
    abstract operator fun get(displacement: Double): Double
    abstract fun deriv(displacement: Double): Double
    abstract fun secondDeriv(displacement: Double): Double
}