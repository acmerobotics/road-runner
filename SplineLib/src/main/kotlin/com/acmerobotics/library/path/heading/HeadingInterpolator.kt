package com.acmerobotics.library.path.heading

import com.acmerobotics.library.path.parametric.QuinticSplineSegment

interface HeadingInterpolator {
    fun init(spline: QuinticSplineSegment)
    operator fun get(displacement: Double): Double
    fun deriv(displacement: Double): Double
    fun secondDeriv(displacement: Double): Double
}