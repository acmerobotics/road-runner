package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator
import org.apache.commons.math3.exception.ConvergenceException
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresBuilder
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer
import org.apache.commons.math3.linear.ArrayRealVector
import org.apache.commons.math3.linear.MatrixUtils
import org.apache.commons.math3.linear.RealMatrix
import org.apache.commons.math3.linear.RealVector


/**
 * Path composed of a parametric curve and a heading interpolator.
 *
 * @param parametricCurve parametric curve
 * @param interpolator heading interpolator
 * @param reversed whether or not to travel along the path in reverse
 */
// TODO: support composite paths?
// or find another way for gvf to follow composite curves
class Path @JvmOverloads constructor(
        val parametricCurve: ParametricCurve,
        val interpolator: HeadingInterpolator = TangentInterpolator(),
        val reversed: Boolean = false
) {
    class ProjectionResult(val displacement: Double, val distance: Double)

    init {
        interpolator.init(parametricCurve)
    }

    /**
     * Returns the length of the path.
     */
    fun length() = parametricCurve.length()

    /**
     * Returns the pose [displacement] units along the path.
     */
    operator fun get(displacement: Double): Pose2d {
        val point = if (reversed) {
            parametricCurve[length() - displacement]
        } else {
            parametricCurve[displacement]
        }
        val heading = if (reversed) {
            interpolator[length() - displacement]
        } else {
            interpolator[displacement]
        }
        return Pose2d(point.x, point.y, heading)
    }

    /**
     * Returns the pose derivative [displacement] units along the path.
     */
    fun deriv(displacement: Double): Pose2d {
        val deriv = if (reversed) {
            -parametricCurve.deriv(length() - displacement)
        } else {
            parametricCurve.deriv(displacement)
        }
        val headingDeriv = if (reversed) {
            -interpolator.deriv(length() - displacement)
        } else {
            interpolator.deriv(displacement)
        }
        return Pose2d(deriv.x, deriv.y, headingDeriv)
    }

    /**
     * Returns the pose second derivative [displacement] units along the path.
     */
    fun secondDeriv(displacement: Double): Pose2d {
        val secondDeriv = if (reversed) {
            parametricCurve.secondDeriv(length() - displacement)
        } else {
            parametricCurve.secondDeriv(displacement)
        }
        val headingSecondDeriv = if (reversed) {
            interpolator.secondDeriv(length() - displacement)
        } else {
            interpolator.secondDeriv(displacement)
        }
        return Pose2d(secondDeriv.x, secondDeriv.y, headingSecondDeriv)
    }

    fun project(point: Vector2d, initialDisplacement: Double = length() / 2.0): ProjectionResult {
        val problem = LeastSquaresBuilder()
                .start(doubleArrayOf(initialDisplacement))
                .model {
                    val pathPoint = this[it.getEntry(0)].pos()
                    val pathDerivative = deriv(it.getEntry(0)).pos()

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