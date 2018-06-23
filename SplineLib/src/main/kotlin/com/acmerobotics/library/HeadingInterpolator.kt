package com.acmerobotics.library

interface HeadingInterpolator {
    operator fun get(displacement: Double): Double
    fun deriv(displacement: Double): Double
    fun secondDeriv(displacement: Double): Double
}