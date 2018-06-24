package com.acmerobotics.library

interface MotionConstraints {
    fun maximumVelocity(displacement: Double): Double
    fun maximumAcceleration(displacement: Double): Double
}