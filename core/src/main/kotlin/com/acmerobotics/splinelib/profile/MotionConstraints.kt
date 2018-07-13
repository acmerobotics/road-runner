package com.acmerobotics.splinelib.profile

interface MotionConstraints {
    fun maximumVelocity(displacement: Double): Double
    fun maximumAcceleration(displacement: Double): Double
}