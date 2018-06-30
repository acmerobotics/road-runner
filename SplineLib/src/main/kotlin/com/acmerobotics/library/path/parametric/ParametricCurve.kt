package com.acmerobotics.library.path.parametric

import com.acmerobotics.library.Vector2d

abstract class ParametricCurve {
    abstract fun length(): Double
    abstract operator fun get(displacement: Double): Vector2d
    abstract fun deriv(displacement: Double): Vector2d
    abstract fun secondDeriv(displacement: Double): Vector2d
    abstract fun thirdDeriv(displacement: Double): Vector2d

    fun start() = get(0.0)

    fun startDeriv() = deriv(0.0)

    fun startSecondDeriv() = secondDeriv(0.0)

    fun startThirdDeriv() = thirdDeriv(0.0)

    fun end() = get(length())

    fun endDeriv() = deriv(length())

    fun endSecondDeriv() = secondDeriv(length())

    fun endThirdDeriv() = thirdDeriv(length())
}