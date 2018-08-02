package com.acmerobotics.splinelib.path.heading

import com.acmerobotics.splinelib.path.ParametricCurve

/**
 * Interface for specifying the heading for holonomic paths.
 */
interface HeadingInterpolator {

    /**
     * Initialize the interpolator with a [parametricCurve].
     *
     *  @param parametricCurve parametric curve
     */
    fun init(parametricCurve: ParametricCurve)

    /**
     * Returns true if the heading interpolator respects derivative continuity at path segment endpoints. That is,
     * the start and end heading derivatives match those of the [TangentInterpolator].
     *
     * @return true if derivatives match [TangentInterpolator]
     */
    fun respectsDerivativeContinuity(): Boolean

    /**
     * Returns the heading at the specified [displacement].
     */
    operator fun get(displacement: Double): Double

    /**
     * Returns the heading derivative at the specified [displacement].
     */
    fun deriv(displacement: Double): Double

    /**
     * Returns the heading second derivative at the specified [displacement].
     */
    fun secondDeriv(displacement: Double): Double
}