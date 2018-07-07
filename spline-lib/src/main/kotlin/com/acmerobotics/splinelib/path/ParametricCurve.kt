package com.acmerobotics.splinelib.path

import com.acmerobotics.splinelib.Vector2d

abstract class ParametricCurve {
    operator fun get(displacement: Double) = internalGet(displacementToParameter(displacement))

    fun deriv(displacement: Double): Vector2d {
        val t = displacementToParameter(displacement)
        return internalDeriv(t) * parameterDeriv(t)
    }

    fun secondDeriv(displacement: Double): Vector2d {
        val t = displacementToParameter(displacement)
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val splineParameterDeriv = parameterDeriv(t)
        val splineParameterSecondDeriv = parameterSecondDeriv(t)
        return secondDeriv * splineParameterDeriv * splineParameterDeriv +
                deriv * splineParameterSecondDeriv
    }

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

    fun start() = get(0.0)

    fun startDeriv() = deriv(0.0)

    fun startSecondDeriv() = secondDeriv(0.0)

    fun startThirdDeriv() = thirdDeriv(0.0)

    fun end() = get(length())

    fun endDeriv() = deriv(length())

    fun endSecondDeriv() = secondDeriv(length())

    fun endThirdDeriv() = thirdDeriv(length())

    private fun internalTangentAngle(t: Double): Double {
        val pathDeriv = internalDeriv(t)
        val angle = Math.atan2(pathDeriv.y, pathDeriv.x)
        return if (angle.isNaN()) 0.0 else angle
    }

    internal fun internalTangentAngleDeriv(t: Double): Double {
        val pathDeriv = internalDeriv(t)
        val pathSecondDeriv = internalSecondDeriv(t)

        var deriv = pathDeriv.x * pathSecondDeriv.y - pathSecondDeriv.x * pathDeriv.y
        deriv /= (pathDeriv.x * pathDeriv.x + pathDeriv.y * pathDeriv.y)

        return if (deriv.isNaN()) 0.0 else deriv
    }

    internal fun internalTangentAngleSecondDeriv(t: Double): Double {
        val pathDeriv = internalDeriv(t)
        val pathSecondDeriv = internalSecondDeriv(t)
        val pathThirdDeriv = internalThirdDeriv(t)

        // if you're curious and hate yourself enough, here's the complete formula:
        // http://www.wolframalpha.com/input/?i=d%2Fds(d%2Fds(arctan((dy%2Fds)%2F(dx%2Fds))))
        val denominator = pathDeriv.x * pathDeriv.x + pathDeriv.y * pathDeriv.y
        val firstTerm = (pathThirdDeriv.y * pathDeriv.x - pathThirdDeriv.x * pathDeriv.y) / denominator
        var secondTerm = (pathDeriv.x * pathSecondDeriv.y - pathSecondDeriv.x * pathDeriv.y)
        secondTerm *= 2 * (pathDeriv.x * pathSecondDeriv.x + pathDeriv.y * pathSecondDeriv.y)
        secondTerm /= (denominator * denominator)
        val secondDeriv = firstTerm - secondTerm

        return if (secondDeriv.isNaN()) 0.0 else secondDeriv
    }

    fun tangentAngle(displacement: Double): Double {
        val t = displacementToParameter(displacement)
        return internalTangentAngle(t)
    }

    fun tangentAngleDeriv(displacement: Double): Double {
        val t = displacementToParameter(displacement)
        return internalTangentAngleDeriv(t) * parameterDeriv(t)
    }

    fun tangentAngleSecondDeriv(displacement: Double): Double {
        val t = displacementToParameter(displacement)
        return internalTangentAngleSecondDeriv(t) * parameterDeriv(t) * parameterDeriv(t) +
                internalTangentAngleDeriv(t) * parameterSecondDeriv(t)
    }

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