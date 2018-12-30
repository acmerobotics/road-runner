package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.Vector2d

/**
 * Parametric curve with two components (x and y).
 */
abstract class ParametricCurve {

    /**
     * Returns the vector [displacement] units along the curve.
     */
    operator fun get(displacement: Double) = internalGet(displacementToParameter(displacement))

    /**
     * Returns the derivative [displacement] units along the curve.
     */
    fun deriv(displacement: Double): Vector2d {
        val t = displacementToParameter(displacement)
        return internalDeriv(t) * parameterDeriv(t)
    }

    /**
     * Returns the second derivative [displacement] units along the curve.
     */
    fun secondDeriv(displacement: Double): Vector2d {
        val t = displacementToParameter(displacement)
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val splineParameterDeriv = parameterDeriv(t)
        val splineParameterSecondDeriv = parameterSecondDeriv(t)
        return secondDeriv * splineParameterDeriv * splineParameterDeriv +
                deriv * splineParameterSecondDeriv
    }

    /**
     * Returns the third derivative [displacement] units along the curve.
     */
    fun thirdDeriv(displacement: Double): Vector2d {
        val t = displacementToParameter(displacement)
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val thirdDeriv = internalThirdDeriv(t)
        val splineParameterDeriv = parameterDeriv(t)
        val splineParameterSecondDeriv = parameterSecondDeriv(t)
        val splineParameterThirdDeriv = parameterThirdDeriv(t)
        return thirdDeriv * splineParameterDeriv * splineParameterDeriv * splineParameterDeriv +
                secondDeriv * splineParameterSecondDeriv * splineParameterDeriv * 3.0 +
                deriv * splineParameterThirdDeriv
    }

    /**
     * Returns the start vector.
     */
    fun start() = get(0.0)

    /**
     * Returns the start derivative.
     */
    fun startDeriv() = deriv(0.0)

    /**
     * Returns the start second derivative.
     */
    fun startSecondDeriv() = secondDeriv(0.0)

    /**
     * Returns the start third derivative.
     */
    fun startThirdDeriv() = thirdDeriv(0.0)

    /**
     * Returns the end vector.
     */
    fun end() = get(length())

    /**
     * Returns the end derivative.
     */
    fun endDeriv() = deriv(length())

    /**
     * Returns the end second derivative.
     */
    fun endSecondDeriv() = secondDeriv(length())

    /**
     * Returns the end third derivative.
     */
    fun endThirdDeriv() = thirdDeriv(length())

    /**
     * Returns the angle of the tangent line [displacement] units along the curve.
     */
    fun tangentAngle(displacement: Double) = deriv(displacement).angle()

    /**
     * Returns the derivative of the tangent angle [displacement] units along the curve.
     */
    fun tangentAngleDeriv(displacement: Double): Double {
        val deriv = deriv(displacement)
        val secondDeriv = secondDeriv(displacement)
        return deriv.x * secondDeriv.y - deriv.y * secondDeriv.x
    }

    /**
     * Returns the second derivative of the tangent angle [displacement] units along the curve.
     */
    fun tangentAngleSecondDeriv(displacement: Double): Double {
        val deriv = deriv(displacement)
        val thirdDeriv = thirdDeriv(displacement)
        return deriv.x * thirdDeriv.y - deriv.y * thirdDeriv.x
    }

    /**
     * Returns the length of the curve.
     */
    abstract fun length(): Double

    internal abstract fun internalGet(t: Double): Vector2d
    internal abstract fun internalDeriv(t: Double): Vector2d
    internal abstract fun internalSecondDeriv(t: Double): Vector2d
    internal abstract fun internalThirdDeriv(t: Double): Vector2d

    internal abstract fun displacementToParameter(displacement: Double): Double
    internal abstract fun parameterDeriv(t: Double): Double
    internal abstract fun parameterSecondDeriv(t: Double): Double
    internal abstract fun parameterThirdDeriv(t: Double): Double
}