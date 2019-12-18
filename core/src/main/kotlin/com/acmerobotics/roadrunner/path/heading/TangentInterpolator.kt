package com.acmerobotics.roadrunner.path.heading

import com.acmerobotics.roadrunner.util.Angle
import kotlin.math.PI

/**
 * Tangent (system) interpolator for tank/differential and other nonholonomic drives.
 *
 * @param reversed
 */
class TangentInterpolator @JvmOverloads constructor(internal val reversed: Boolean = false) : HeadingInterpolator() {
    override fun internalGet(s: Double, t: Double): Double {
        val angle = curve.tangentAngle(s, t)
        return if (reversed) {
            Angle.norm(angle + PI)
        } else {
            angle
        }
    }

    override fun internalDeriv(s: Double, t: Double) = curve.tangentAngleDeriv(s, t)

    override fun internalSecondDeriv(s: Double, t: Double) = curve.tangentAngleSecondDeriv(s, t)
}
