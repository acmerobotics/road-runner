package com.acmerobotics.library

interface HeadingInterpolator {
    fun init(path: Path)
    operator fun get(displacement: Double): Double
    fun deriv(displacement: Double): Double
    fun secondDeriv(displacement: Double): Double
}