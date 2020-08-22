package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.DoubleProgression

/**
 * Path composed of a list of parametric curves and heading interpolators.
 *
 * @param segments list of path segments
 */
class Path(val segments: List<PathSegment>) {
    private val projector = PathProjector(this)

    /**
     * @param segment single path segment
     */
    constructor(segment: PathSegment) : this(listOf(segment))

    /**
     * Returns the length of the path.
     */
    fun length() = segments.sumByDouble { it.length() }

    fun segment(s: Double): Pair<PathSegment, Double> {
        if (s <= 0.0) {
            return segments.first() to 0.0
        }
        var remainingDisplacement = s
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment to remainingDisplacement
            }
            remainingDisplacement -= segment.length()
        }
        return segments.last() to segments.last().length()
    }

    /**
     * Returns the pose [s] units along the path.
     */
    @JvmOverloads
    operator fun get(s: Double, t: Double = reparam(s)): Pose2d {
        val (segment, remainingDisplacement) = segment(s)
        return segment[remainingDisplacement, t]
    }

    /**
     * Returns the pose derivative [s] units along the path.
     */
    @JvmOverloads
    fun deriv(s: Double, t: Double = reparam(s)): Pose2d {
        val (segment, remainingDisplacement) = segment(s)
        return segment.deriv(remainingDisplacement, t)
    }

    /**
     * Returns the pose second derivative [s] units along the path.
     */
    @JvmOverloads
    fun secondDeriv(s: Double, t: Double = reparam(s)): Pose2d {
        val (segment, remainingDisplacement) = segment(s)
        return segment.secondDeriv(remainingDisplacement, t)
    }

    @JvmOverloads
    internal fun internalDeriv(s: Double, t: Double = reparam(s)): Pose2d {
        val (segment, remainingDisplacement) = segment(s)
        return segment.internalDeriv(remainingDisplacement, t)
    }

    @JvmOverloads
    internal fun internalSecondDeriv(s: Double, t: Double = reparam(s)): Pose2d {
        val (segment, remainingDisplacement) = segment(s)
        return segment.internalSecondDeriv(remainingDisplacement, t)
    }

    internal fun reparam(s: Double): Double {
        val (segment, remainingDisplacement) = segment(s)
        return segment.reparam(remainingDisplacement)
    }

    internal fun reparam(s: DoubleProgression): DoubleArray {
        val t = DoubleArray(s.size())
        // skip any negative s entries (the corresponding array entries are already 0.0)
        var (ignore, remainingDisplacement) = s.split(0.0)
        var offset = ignore.size()
        for (segment in segments) {
            if (offset == t.size) {
                break
            }
            val pair =
                    remainingDisplacement.split(segment.length())
            val segmentDisplacement = pair.first
            if (!segmentDisplacement.isEmpty()) {
                segment.reparam(segmentDisplacement).copyInto(t, offset, 0)
                offset += segmentDisplacement.size()
            }
            remainingDisplacement = pair.second - segment.length()
        }
        while (offset < t.size) {
            t[offset++] = 1.0
        }
        return t
    }

    /**
     * Based on the guess, this method converges to the nearest orthogonal projection. See [PathProjector] for details.
     *
     * @param queryPoint query queryPoint
     * @param projectGuess guess for the projected queryPoint's s along the path
     */
    fun fastProject(queryPoint: Vector2d, projectGuess: Double = length() / 2.0) =
            projector.localProject(queryPoint, projectGuess)

    /**
     * Finds the nearest point on the path to the query point. See [PathProjector] for details.
     *
     * @param queryPoint query queryPoint
     */
    fun project(queryPoint: Vector2d) = projector.project(queryPoint)

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
