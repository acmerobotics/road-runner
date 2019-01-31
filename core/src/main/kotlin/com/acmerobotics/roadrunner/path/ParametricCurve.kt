package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.Vector2d

/**
 * Parametric curve with two components (x and y).
 */
abstract class ParametricCurve {

    /**
     * Returns the vector [s] units along the curve.
     */
    operator fun get(s: Double) = internalGet(reparam(s))

    /**
     * Returns the derivative [s] units along the curve.
     */
    fun deriv(s: Double): Vector2d {
        val t = reparam(s)
        return internalDeriv(t) * paramDeriv(t)
    }

    /**
     * Returns the second derivative [s] units along the curve.
     */
    fun secondDeriv(s: Double): Vector2d {
        val t = reparam(s)
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val splineParameterDeriv = paramDeriv(t)
        val splineParameterSecondDeriv = paramSecondDeriv(t)
        return secondDeriv * splineParameterDeriv * splineParameterDeriv +
                deriv * splineParameterSecondDeriv
    }

    /**
     * Returns the third derivative [s] units along the curve.
     */
    fun thirdDeriv(s: Double): Vector2d {
        val t = reparam(s)
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val thirdDeriv = internalThirdDeriv(t)
        val splineParameterDeriv = paramDeriv(t)
        val splineParameterSecondDeriv = paramSecondDeriv(t)
        val splineParameterThirdDeriv = paramThirdDeriv(t)
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
     * Returns the angle of the tangent line [s] units along the curve.
     */
    fun tangentAngle(s: Double) = deriv(s).angle()

    /**
     * Returns the derivative of the tangent angle [s] units along the curve.
     */
    fun tangentAngleDeriv(s: Double): Double {
        val deriv = deriv(s)
        val secondDeriv = secondDeriv(s)
        return deriv.x * secondDeriv.y - deriv.y * secondDeriv.x
    }

    /**
     * Returns the second derivative of the tangent angle [s] units along the curve.
     */
    fun tangentAngleSecondDeriv(s: Double): Double {
        val deriv = deriv(s)
        val thirdDeriv = thirdDeriv(s)
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

    internal abstract fun reparam(s: Double): Double
    internal abstract fun paramDeriv(t: Double): Double
    internal abstract fun paramSecondDeriv(t: Double): Double
    internal abstract fun paramThirdDeriv(t: Double): Double
}