package com.acmerobotics.roadrunner.profile

/**
 * Constant velocity and acceleration constraints used by [MotionProfileGenerator.generateSimpleMotionProfile].
 *
 * @param maximumVelocity constant maximum velocity
 * @param maximumAcceleration constant maximum acceleration
 */
class SimpleMotionConstraints(
        @JvmField var maximumVelocity: Double,
        @JvmField var maximumAcceleration: Double
) : MotionConstraints {

    override fun maximumVelocity(displacement: Double) = maximumVelocity

    override fun maximumAcceleration(displacement: Double) = maximumAcceleration
}