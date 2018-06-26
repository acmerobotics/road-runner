package com.acmerobotics.library.path

import com.acmerobotics.library.Vector2d

abstract class Path {
    abstract fun length(): Double
    abstract operator fun get(displacement: Double): Vector2d
    abstract fun deriv(displacement: Double): Vector2d
    abstract fun secondDeriv(displacement: Double): Vector2d
    abstract fun thirdDeriv(displacement: Double): Vector2d
}