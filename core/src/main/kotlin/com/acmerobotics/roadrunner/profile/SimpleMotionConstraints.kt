package com.acmerobotics.roadrunner.profile

class SimpleMotionConstraints(
        @JvmField var maximumVelocity: Double,
        @JvmField var maximumAcceleration: Double
) : MotionConstraints {

    override fun maximumVelocity(displacement: Double) = maximumVelocity

    override fun maximumAcceleration(displacement: Double) = maximumAcceleration
}