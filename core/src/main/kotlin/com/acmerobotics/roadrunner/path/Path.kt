package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.DoubleProgression
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
 * @param segments list of path segments
 */
class Path @JvmOverloads constructor(val segments: List<PathSegment> = emptyList()) {

    /**
     * @param segment single path segment
     */
    constructor(segment: PathSegment) : this(listOf(segment))

    /**
     * Returns the length of the path.
     */
    fun length() = segments.sumByDouble { it.length() }

    /**
     * Returns the pose [s] units along the path.
     */
    @JvmOverloads
    operator fun get(s: Double, t: Double = reparam(s)): Pose2d {
        if (s <= 0.0) {
            return segments.firstOrNull()?.start() ?: return Pose2d()
        }
        var remainingDisplacement = s
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment[remainingDisplacement, t]
            }
            remainingDisplacement -= segment.length()
        }
        return segments.lastOrNull()?.end() ?: return Pose2d()
    }

    /**
     * Returns the pose derivative [s] units along the path.
     */
    @JvmOverloads
    fun deriv(s: Double, t: Double = reparam(s)): Pose2d {
        if (s <= 0.0) {
            return segments.firstOrNull()?.startDeriv() ?: return Pose2d()
        }
        var remainingDisplacement = s
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment.deriv(remainingDisplacement, t)
            }
            remainingDisplacement -= segment.length()
        }
        return segments.lastOrNull()?.endDeriv() ?: return Pose2d()
    }

    /**
     * Returns the pose second derivative [s] units along the path.
     */
    @JvmOverloads
    fun secondDeriv(s: Double, t: Double = reparam(s)): Pose2d {
        if (s <= 0.0) {
            return segments.firstOrNull()?.startSecondDeriv() ?: return Pose2d()
        }
        var remainingDisplacement = s
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment.secondDeriv(remainingDisplacement, t)
            }
            remainingDisplacement -= segment.length()
        }
        return segments.lastOrNull()?.endSecondDeriv() ?: return Pose2d()
    }

    @JvmOverloads
    internal fun internalDeriv(s: Double, t: Double = reparam(s)): Pose2d {
        if (s <= 0.0) {
            return segments.firstOrNull()?.startInternalDeriv() ?: return Pose2d()
        }
        var remainingDisplacement = s
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment.internalDeriv(remainingDisplacement, t)
            }
            remainingDisplacement -= segment.length()
        }
        return segments.lastOrNull()?.endInternalDeriv() ?: return Pose2d()
    }

    @JvmOverloads
    internal fun internalSecondDeriv(s: Double, t: Double = reparam(s)): Pose2d {
        if (s <= 0.0) {
            return segments.firstOrNull()?.startInternalSecondDeriv() ?: return Pose2d()
        }
        var remainingDisplacement = s
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment.internalSecondDeriv(remainingDisplacement, t)
            }
            remainingDisplacement -= segment.length()
        }
        return segments.lastOrNull()?.endInternalSecondDeriv() ?: return Pose2d()
    }

    internal fun reparam(s: Double): Double {
        if (s <= 0.0) {
            return 0.0
        }
        var remainingDisplacement = s
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment.reparam(remainingDisplacement)
            }
            remainingDisplacement -= segment.length()
        }
        return 1.0
    }

    internal fun reparam(s: DoubleProgression): DoubleArray {
        val t = DoubleArray(s.items())
        // skip any negative s entries (the corresponding array entries are already 0.0)
        var (ignore, remainingDisplacement) = s.split(0.0)
        var offset = ignore.items()
        for (segment in segments) {
            val pair =
                remainingDisplacement.split(segment.length())
            val segmentDisplacement = pair.first
            if (!segmentDisplacement.isEmpty()) {
                segment.reparam(segmentDisplacement).copyInto(t, offset, 0)
                offset += segmentDisplacement.items()
            }
            remainingDisplacement = pair.second - segment.length()
        }
        while (offset < t.size) {
            t[offset++] = 1.0
        }
        return t
    }

    /**
     * Project [point] onto the current path.
     *
     * @param point query point
     * @param projectGuess guess for the projected point's s along the path
     */
    // TODO: use something more specialized than Levenberg-Marquardt?
    fun project(point: Vector2d, projectGuess: Double = length() / 2.0): Double {
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

        return try {
            val optimum = LevenbergMarquardtOptimizer().optimize(problem)
            optimum.point.getEntry(0)
        } catch (e: ConvergenceException) {
            0.0
        }
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
