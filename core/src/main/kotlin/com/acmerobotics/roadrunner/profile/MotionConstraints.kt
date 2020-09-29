package com.acmerobotics.roadrunner.profile

/**
 * Motion profile motion constraints.
 */
abstract class MotionConstraints {

    /**
     * Returns the motion constraints [s] units along the profile.
     */
    abstract operator fun get(s: Double): SimpleMotionConstraints
}
