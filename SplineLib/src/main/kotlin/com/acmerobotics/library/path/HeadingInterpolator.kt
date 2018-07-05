package com.acmerobotics.library.path

interface HeadingInterpolator {
    fun init(splineSegment: QuinticSplineSegment)
    operator fun get(displacement: Double): Double
    fun deriv(displacement: Double): Double
    fun secondDeriv(displacement: Double): Double
}