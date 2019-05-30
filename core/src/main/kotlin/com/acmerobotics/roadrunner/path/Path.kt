package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.DoubleProgression
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.abs
import kotlin.math.sign

/**
 * Path composed of a list of parametric curves and heading interpolators.
 *
 * @param segments list of path segments
 */
class Path(val segments: List<PathSegment>) {

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
     * Project [queryPoint] onto the current path using the iterative method described
     * [here](http://www.geometrie.tugraz.at/wallner/sproj.pdf).
     *
     * @param queryPoint query queryPoint
     * @param projectGuess guess for the projected queryPoint's s along the path
     */
    fun project(queryPoint: Vector2d, projectGuess: Double = length() / 2.0): Double {
        var s = projectGuess
        while (true) {
            val pathPoint = get(s).pos()
            val deriv = deriv(s).pos()
            val secondDeriv = deriv(s).pos()
            val k = secondDeriv.norm()
            val ds = if (k epsilonEquals 0.0) {
                // use the first-order method
                // this should always work as derivNorm = 1.0 for arc length param
                // (and generally derivNorm != 0.0 for smooth params)

                // for first-order, qRel is the projection onto the tangent
                val qRel = (queryPoint - pathPoint) projectOnto deriv
                deriv dot qRel
            } else {
                // use the second-order method

                // for second-order, qRel is the projection onto the osculating circle
                val pToOrigin = queryPoint - pathPoint - secondDeriv
                val qRel = secondDeriv + pToOrigin / pToOrigin.norm() * k
                val area = abs(deriv.x * qRel.y - qRel.x * deriv.y)
                2.0 * area / k * sign(deriv dot qRel)
            }

            if (ds epsilonEquals 0.0) break

            s += ds
        }
        return s
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
