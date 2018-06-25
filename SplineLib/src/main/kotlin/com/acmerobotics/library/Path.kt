package com.acmerobotics.library

abstract class Path(val motionConstraints: PathMotionConstraints) {
    abstract fun length(): Double
    abstract operator fun get(displacement: Double): Vector2d
    abstract fun deriv(displacement: Double): Vector2d
    abstract fun secondDeriv(displacement: Double): Vector2d
    abstract fun thirdDeriv(displacement: Double): Vector2d
}