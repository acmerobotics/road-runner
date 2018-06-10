package com.acmerobotics.library

import kotlin.math.sqrt

class MotionState(val x: Double, val v: Double, val a: Double) {
    fun afterDisplacement(dx: Double) = MotionState(x + dx, sqrt(v * v + 2 * a * dx), a)
    override fun toString() = String.format("x=%.3f, v=%.3f, a=%.3f", x, v, a)
}