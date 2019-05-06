package com.acmerobotics.roadrunner.profile

/**
 * Constant velocity and acceleration constraints.
 *
 * @param maxVel constant maximum velocity
 * @param maxAccel constant maximum acceleration
 */
data class SimpleMotionConstraints(
    @JvmField var maxVel: Double,
    @JvmField var maxAccel: Double
)
