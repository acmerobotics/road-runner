package com.acmerobotics.roadrunner.path.heading

/**
 * Linear heading interpolator for time-optimal transitions between poses.
 *
 * @param startHeading start heading
 * @param endHeading end heading
 */
class LinearInterpolator(private val startHeading: Double, endHeading: Double) : HeadingInterpolator() {
    private val turnAngle: Double = if (endHeading >= startHeading) {
        endHeading - startHeading
    } else {
        2 * Math.PI - endHeading + startHeading
    }

    override fun respectsDerivativeContinuity() = false

    override fun internalGet(s: Double, t: Double) =
        (startHeading + s / parametricCurve.length() * turnAngle) % (2 * Math.PI)

    override fun internalDeriv(s: Double, t: Double) = turnAngle / parametricCurve.length()

    override fun internalSecondDeriv(s: Double, t: Double) = 0.0
}