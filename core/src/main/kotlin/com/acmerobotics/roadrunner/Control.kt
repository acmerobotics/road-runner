package com.acmerobotics.roadrunner

import kotlin.math.withSign

class Feedforward(
    val kS: Double,
    val kV: Double,
    val kA: Double,
) {
    fun compute(vel: Double, accel: Double): Double {
        val basePower = vel * kV + accel * kA
        return if (basePower == 0.0) {
            0.0
        } else {
            basePower + kS.withSign(basePower)
        }
    }
}
