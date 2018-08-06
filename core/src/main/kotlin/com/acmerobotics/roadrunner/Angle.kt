package com.acmerobotics.roadrunner

/**
 * Various utilities for working with angles.
 */
object Angle {
    private const val TAU = Math.PI * 2

    /**
     * Returns the normalized version of (clamped to (-pi, pi)) [angle].
     *
     * @param angle angle measure in radians
     */
    @JvmStatic
    fun norm(angle: Double): Double {
        var modifiedAngle = angle % TAU

        modifiedAngle = (modifiedAngle + TAU) % TAU

        if (modifiedAngle > Math.PI) {
            modifiedAngle -= TAU
        }

        return modifiedAngle
    }

}