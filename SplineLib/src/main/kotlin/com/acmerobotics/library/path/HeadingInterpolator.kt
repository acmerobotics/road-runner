package com.acmerobotics.library.path

interface HeadingInterpolator {
    fun init(parametricCurve: ParametricCurve)
    operator fun get(displacement: Double): Double
    fun deriv(displacement: Double): Double
    fun secondDeriv(displacement: Double): Double
}