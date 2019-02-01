package com.acmerobotics.roadrunner.profile

/**
 * Constant velocity and acceleration constraints.
 *
 * @param maximumVelocity constant maximum velocity
 * @param maximumAcceleration constant maximum acceleration
 */
data class SimpleMotionConstraints(
    @JvmField var maximumVelocity: Double,
    @JvmField var maximumAcceleration: Double
)