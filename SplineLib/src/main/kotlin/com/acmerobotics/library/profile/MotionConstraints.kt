package com.acmerobotics.library.profile

interface MotionConstraints {
    fun maximumVelocity(displacement: Double): Double
    fun maximumAcceleration(displacement: Double): Double
}