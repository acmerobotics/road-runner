package com.acmerobotics.roadrunner.util

import kotlin.math.PI

/**
 * Various utilities for working with angles.
 */
object Angle {
    private const val TAU = PI * 2

    /**
     * Returns [angle] clamped to `[-pi, pi]`.
     *
     * @param angle angle measure in radians
     */
    @JvmStatic
    fun norm(angle: Double): Double {
        var modifiedAngle = angle % TAU

        modifiedAngle = (modifiedAngle + TAU) % TAU

        if (modifiedAngle > PI) {
            modifiedAngle -= TAU
        }

        return modifiedAngle
    }
}
