package com.acmerobotics.roadrunner.profile

import com.acmerobotics.roadrunner.util.DoubleProgression

/**
 * Motion profile motion constraints.
 */
abstract class MotionConstraints {

    /**
     * Returns the motion constraints [s] units along the profile.
     */
    abstract operator fun get(s: Double): SimpleMotionConstraints

    open operator fun get(s: DoubleProgression): List<SimpleMotionConstraints> = s.map(::get)
}