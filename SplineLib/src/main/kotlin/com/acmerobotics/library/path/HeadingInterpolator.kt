package com.acmerobotics.library.path

abstract class HeadingInterpolator {
    protected lateinit var path: Path

    open fun init(path: Path) {
        this.path = path
    }

    abstract operator fun get(displacement: Double): Double
    abstract fun deriv(displacement: Double): Double
    abstract fun secondDeriv(displacement: Double): Double
}