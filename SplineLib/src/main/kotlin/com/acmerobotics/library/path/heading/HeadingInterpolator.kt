package com.acmerobotics.library.path.heading

import com.acmerobotics.library.path.parametric.ParametricCurve

abstract class HeadingInterpolator {
    protected lateinit var parametricCurve: ParametricCurve

    open fun init(parametricCurve: ParametricCurve) {
        this.parametricCurve = parametricCurve
    }

    abstract operator fun get(displacement: Double): Double
    abstract fun deriv(displacement: Double): Double
    abstract fun secondDeriv(displacement: Double): Double
}