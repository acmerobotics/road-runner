package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator
import com.acmerobotics.roadrunner.util.DoubleProgression
import com.acmerobotics.roadrunner.util.minus
import org.apache.commons.math3.exception.ConvergenceException
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresBuilder
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer
import org.apache.commons.math3.linear.ArrayRealVector
import org.apache.commons.math3.linear.MatrixUtils
import org.apache.commons.math3.linear.RealMatrix
import org.apache.commons.math3.linear.RealVector

/**
 * Path composed of a list of parametric curves and heading interpolators.
 *
 * @param parametricCurves parametric curves
 * @param interpolators heading interpolators
 * @param reversed whether or not to travel along the path segment in reverse
 */
class Path @JvmOverloads constructor(
    val parametricCurves: List<ParametricCurve> = emptyList(),
    val interpolators: List<HeadingInterpolator> = parametricCurves.map { TangentInterpolator() },
    val reversed: List<Boolean> = parametricCurves.map { false }
) {
    /**
     * @param parametricCurve parametric curve
     * @param interpolator heading interpolator
     * @param reversed whether or not to travel in reverse
     */
    @JvmOverloads constructor(
        parametricCurve: ParametricCurve,
        interpolator: HeadingInterpolator = TangentInterpolator(),
        reversed: Boolean = false
    ) : this(listOf(parametricCurve), listOf(interpolator), listOf(reversed))

    /**
     * Simple container for the result of a projection (i.e., a [project] call).
     *
     * @param displacement displacement along the path
     * @param distance Euclidean distance between the path and query points
     */
    data class ProjectionResult(val displacement: Double, val distance: Double)

    init {
        interpolators.zip(parametricCurves).forEach { it.first.init(it.second) }
    }

    /**
     * Returns the length of the path.
     */
    fun length() = parametricCurves.sumByDouble { it.length() }

    /**
     * Returns the pose [s] units along the path.
     */
    @JvmOverloads
    operator fun get(s: Double, t: Double = reparam(s)): Pose2d {
        var remainingDisplacement = s
        for (i in parametricCurves.indices) {
            val parametricCurve = parametricCurves[i]
            if (remainingDisplacement <= parametricCurve.length()) {
                return segmentGet(i, remainingDisplacement, t)
            }
            remainingDisplacement -= parametricCurve.length()
        }
        val finalVector = parametricCurves.lastOrNull()?.end() ?: return Pose2d()
        return Pose2d(finalVector, interpolators.last().end())
    }

    /**
     * Returns the pose derivative [s] units along the path.
     */
    @JvmOverloads
    fun deriv(s: Double, t: Double = reparam(s)): Pose2d {
        var remainingDisplacement = s
        for (i in parametricCurves.indices) {
            val parametricCurve = parametricCurves[i]
            if (remainingDisplacement <= parametricCurve.length()) {
                return segmentDeriv(i, remainingDisplacement, t)
            }
            remainingDisplacement -= parametricCurve.length()
        }
        val finalVector = parametricCurves.lastOrNull()?.end() ?: return Pose2d()
        return Pose2d(finalVector, interpolators.last().end())
    }

    /**
     * Returns the pose second derivative [s] units along the path.
     */
    @JvmOverloads
    fun secondDeriv(s: Double, t: Double = reparam(s)): Pose2d {
        var remainingDisplacement = s
        for (i in parametricCurves.indices) {
            val parametricCurve = parametricCurves[i]
            if (remainingDisplacement <= parametricCurve.length()) {
                return segmentSecondDeriv(i, remainingDisplacement, t)
            }
            remainingDisplacement -= parametricCurve.length()
        }
        val finalVector = parametricCurves.lastOrNull()?.end() ?: return Pose2d()
        return Pose2d(finalVector, interpolators.last().end())
    }

    internal fun reparam(s: Double): Double {
        var remainingDisplacement = s
        for (i in parametricCurves.indices) {
            val parametricCurve = parametricCurves[i]
            if (remainingDisplacement <= parametricCurve.length()) {
                return segmentReparam(i, remainingDisplacement)
            }
            remainingDisplacement -= parametricCurve.length()
        }
        return if (reversed.last()) {
            0.0
        } else {
            1.0
        }
    }

    internal fun reparam(s: DoubleProgression): DoubleArray {
        val t = DoubleArray(s.items())
        var offset = 0
        var remainingDisplacement = s
        for (i in parametricCurves.indices) {
            val parametricCurve = parametricCurves[i]
            val splitDisplacement =
                remainingDisplacement.split(parametricCurve.length())
            val segmentDisplacement = if (i == parametricCurves.lastIndex) {
                remainingDisplacement
            } else {
                splitDisplacement.first
            }
            if (!segmentDisplacement.isEmpty()) {
                segmentReparam(i, segmentDisplacement).copyInto(t, offset, 0)
                offset += segmentDisplacement.items()
            }
            remainingDisplacement = splitDisplacement.second - parametricCurve.length()
        }
        return t
    }

    private fun segmentGet(i: Int, s: Double, t: Double): Pose2d {
        val parametricCurve = parametricCurves[i]
        val interpolator = interpolators[i]
        val reversed = reversed[i]
        val point = if (reversed) {
            parametricCurve[parametricCurve.length() - s, t]
        } else {
            parametricCurve[s, t]
        }
        val heading = if (reversed) {
            interpolator[parametricCurve.length() - s, t]
        } else {
            interpolator[s, t]
        }
        return Pose2d(point.x, point.y, heading)
    }

    private fun segmentDeriv(i: Int, s: Double, t: Double): Pose2d {
        val parametricCurve = parametricCurves[i]
        val interpolator = interpolators[i]
        val reversed = reversed[i]
        val deriv = if (reversed) {
            -parametricCurve.deriv(parametricCurve.length() - s, t)
        } else {
            parametricCurve.deriv(s, t)
        }
        val headingDeriv = if (reversed) {
            -interpolator.deriv(parametricCurve.length() - s, t)
        } else {
            interpolator.deriv(s, t)
        }
        return Pose2d(deriv.x, deriv.y, headingDeriv)
    }

    private fun segmentSecondDeriv(i: Int, s: Double, t: Double): Pose2d {
        val parametricCurve = parametricCurves[i]
        val interpolator = interpolators[i]
        val reversed = reversed[i]
        val secondDeriv = if (reversed) {
            parametricCurve.secondDeriv(parametricCurve.length() - s, t)
        } else {
            parametricCurve.secondDeriv(s, t)
        }
        val headingSecondDeriv = if (reversed) {
            interpolator.secondDeriv(parametricCurve.length() - s, t)
        } else {
            interpolator.secondDeriv(s, t)
        }
        return Pose2d(secondDeriv.x, secondDeriv.y, headingSecondDeriv)
    }

    private fun segmentReparam(i: Int, s: Double): Double {
        val parametricCurve = parametricCurves[i]
        val reversed = reversed[i]
        return if (reversed) {
            parametricCurve.reparam(parametricCurve.length() - s)
        } else {
            parametricCurve.reparam(s)
        }
    }

    private fun segmentReparam(i: Int, s: DoubleProgression): DoubleArray {
        val parametricCurve = parametricCurves[i]
        val reversed = reversed[i]
        return if (reversed) {
            parametricCurve.reparam(parametricCurve.length() - s)
        } else {
            parametricCurve.reparam(s)
        }
    }

    /**
     * Project [point] onto the current path.
     *
     * @param point query point
     * @param projectGuess guess for the projected point's s along the path
     */
    fun project(point: Vector2d, projectGuess: Double = length() / 2.0): ProjectionResult {
        val problem = LeastSquaresBuilder()
                .start(doubleArrayOf(projectGuess))
                .model { vector ->
                    val pathPoint = this[vector.getEntry(0)].pos()
                    val pathDerivative = deriv(vector.getEntry(0)).pos()

                    val diff = pathPoint - point
                    val distance = diff.norm()

                    val value = ArrayRealVector(doubleArrayOf(distance))

                    val derivative = (diff.x * pathDerivative.x + diff.y * pathDerivative.y) / distance
                    val jacobian = MatrixUtils.createRealMatrix(arrayOf(doubleArrayOf(derivative)))

                    org.apache.commons.math3.util.Pair<RealVector, RealMatrix>(value, jacobian)
                }
                .target(doubleArrayOf(0.0))
                .lazyEvaluation(false)
                .maxEvaluations(1000)
                .maxIterations(1000)
                .build()

        val displacement = try {
            val optimum = LevenbergMarquardtOptimizer().optimize(problem)
            optimum.point.getEntry(0)
        } catch (e: ConvergenceException) {
            0.0
        }
        val optimumPoint = this[displacement].pos()

        return ProjectionResult(displacement, point distanceTo optimumPoint)
    }

    /**
     * Returns the start pose.
     */
    fun start() = get(0.0)

    /**
     * Returns the start pose derivative.
     */
    fun startDeriv() = deriv(0.0)

    /**
     * Returns the start pose second derivative.
     */
    fun startSecondDeriv() = secondDeriv(0.0)

    /**
     * Returns the end pose.
     */
    fun end() = get(length())

    /**
     * Returns the end pose derivative.
     */
    fun endDeriv() = deriv(length())

    /**
     * Returns the end pose second derivative.
     */
    fun endSecondDeriv() = secondDeriv(length())
}
