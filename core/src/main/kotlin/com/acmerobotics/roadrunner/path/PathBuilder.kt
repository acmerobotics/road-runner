package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator
import com.acmerobotics.roadrunner.util.Angle
import java.util.*
import kotlin.math.PI

/**
 * Exception thrown by [PathBuilder].
 */
abstract class PathBuilderException : RuntimeException()

/**
 * Exception thrown when [PathBuilder] methods are chained illegally. This commonly arises when switching from
 * non-tangent interpolation back to tangent interpolation and when splicing paths.
 */
class PathContinuityViolationException : PathBuilderException()

/**
 * Exception thrown when empty path segments are requested.
 */
class EmptyPathSegmentException : PathBuilderException()

private typealias Cont = ((Vector2d) -> Vector2d, (Double) -> Double) -> Pair<Pair<Pose2d?, Double?>, List<PathSegment>>

/**
 * Easy-to-use builder for creating [Path] instances.
 *
 * @param startPose start pose
 * @param startTangent start tangent
 * @param path previous path
 * @param s displacement in previous path
 */
class PathBuilder private constructor(
    startPose: Pose2d?,
    startTangent: Double?,
    internal val path: Path?,
    internal val s: Double?
) {
    @JvmOverloads
    constructor(startPose: Pose2d, startTangent: Double = startPose.heading) :
        this(startPose, startTangent, null, null)

    constructor(startPose: Pose2d, reversed: Boolean) :
        this(startPose, Angle.norm(startPose.heading + if (reversed) PI else 0.0))

    constructor(path: Path, s: Double) : this(null, null, path, s)

    private var cont: Cont = { f, g ->
        Pair(
            Pair(
                startPose?.let { Pose2d(f(it.vec()), g(it.heading)) },
                startTangent?.let(g)
            ), emptyList()
        )
    }

    private fun makeLine(start: Vector2d?, end: Vector2d): LineSegment {
        val start = start ?: path!![s!!].vec()
        if (start epsilonEquals end) {
            throw EmptyPathSegmentException()
        }

        return LineSegment(start, end)
    }

    private fun makeSpline(
        startPosition: Vector2d?, startTangent: Double?,
        endPosition: Vector2d, endTangent: Double
    ): QuinticSpline {
        val startPosition = startPosition ?: path!![s!!].vec()
        if (startPosition epsilonEquals endPosition) {
            throw EmptyPathSegmentException()
        }

        val derivMag = startPosition distTo endPosition
        val startWaypoint = if (startTangent == null) {
            val startDeriv = path!!.internalDeriv(s!!).vec()
            val startSecondDeriv = path.internalSecondDeriv(s).vec()
            QuinticSpline.Knot(startPosition, startDeriv, startSecondDeriv)
        } else {
            QuinticSpline.Knot(startPosition, Vector2d.polar(derivMag, startTangent))
        }
        val endWaypoint = QuinticSpline.Knot(endPosition, Vector2d.polar(derivMag, endTangent))

        return QuinticSpline(startWaypoint, endWaypoint)
    }

    private fun makeTangentInterpolator(startHeading: Double?, startTangent: Double?, curve: ParametricCurve): TangentInterpolator {
        val startHeading = startHeading ?: path!![s!!].heading
        // TODO: Path should probably have tangentAngle()
        val startTangent = startTangent ?: path!!.deriv(s!!).vec().angle()

        val interpolator = TangentInterpolator(startHeading - startTangent)
        interpolator.init(curve)

        return interpolator
    }

    private fun makeConstantInterpolator(heading: Double?) =
        ConstantInterpolator(heading ?: path!![s!!].heading)

    private fun makeLinearInterpolator(startHeading: Double?, endHeading: Double) : LinearInterpolator {
        val startHeading = startHeading ?: path!![s!!].heading
        // TODO: missing init()?
        return LinearInterpolator(startHeading, Angle.normDelta(endHeading - startHeading))
    }

    private fun makeSplineInterpolator(startHeading: Double?, endHeading: Double): SplineInterpolator {
        return if (startHeading == null) {
            SplineInterpolator(
                path!![s!!].heading,
                endHeading,
                path.deriv(s).heading,
                path.secondDeriv(s).heading,
                null,
                null
            )
        } else {
            SplineInterpolator(startHeading, endHeading)
        }
    }

    private fun addSegment(make: ((Vector2d) -> Vector2d, (Double) -> Double, Pose2d?, Double?) -> PathSegment): PathBuilder {
        // TODO: mutable captures suck!
        val k = cont
        cont = { f, g ->
            val (lastConfig, lastSegments) = k(f, g)
            val (lastPose, lastTangent) = lastConfig
            val seg = make(f, g, lastPose, lastTangent)
            Pair(Pair(seg.end(), seg.endTangentAngle()), lastSegments + listOf(seg))
        }

        return this
    }

    /**
     * Adds a line segment with tangent heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineTo(endPosition: Vector2d) = addSegment { f, _, lastPose, lastTangent ->
        val line = makeLine(lastPose?.vec(), f(endPosition))
        val interpolator = makeTangentInterpolator(lastPose?.heading, lastTangent, line)
        PathSegment(line, interpolator)
    }

    /**
     * Adds a line segment with constant heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineToConstantHeading(endPosition: Vector2d) = addSegment { f, _, lastPose, _ ->
        PathSegment(
            makeLine(lastPose?.vec(), f(endPosition)),
            makeConstantInterpolator(lastPose?.heading)
        )
    }

    /**
     * Adds a strafe segment (i.e., a line segment with constant heading interpolation).
     *
     * @param endPosition end position
     */
    fun strafeTo(endPosition: Vector2d) = lineToConstantHeading(endPosition)

    /**
     * Adds a line segment with linear heading interpolation.
     *
     * @param endPose end pose
     */
    fun lineToLinearHeading(endPose: Pose2d) = addSegment { f, g, lastPose, _ ->
        PathSegment(
            makeLine(lastPose?.vec(), f(endPose.vec())),
            makeLinearInterpolator(lastPose?.heading, g(endPose.heading))
        )
    }

    /**
     * Adds a line segment with spline heading interpolation.
     *
     * @param endPose end pose
     */
    fun lineToSplineHeading(endPose: Pose2d) = addSegment { f, g, lastPose, _ ->
        PathSegment(
            makeLine(lastPose?.vec(), f(endPose.vec())),
            makeSplineInterpolator(lastPose?.heading, g(endPose.heading))
        )
    }

    /**
     * Adds a line straight forward.
     *
     * @param distance distance to travel forward
     */
    fun forward(distance: Double): PathBuilder = TODO()

    /**
     * Adds a line straight backward.
     *
     * @param distance distance to travel backward
     */
    fun back(distance: Double): PathBuilder {
        forward(-distance)
        return this
    }

    /**
     * Adds a segment that strafes left in the robot reference frame.
     *
     * @param distance distance to strafe left
     */
    fun strafeLeft(distance: Double): PathBuilder = TODO()

    /**
     * Adds a segment that strafes right in the robot reference frame.
     *
     * @param distance distance to strafe right
     */
    fun strafeRight(distance: Double): PathBuilder {
        return strafeLeft(-distance)
    }

    /**
     * Adds a spline segment with tangent heading interpolation.
     *
     * @param endPosition end position
     * @param endTangent end tangent
     */
    fun splineTo(endPosition: Vector2d, endTangent: Double) = addSegment { f, g, lastPose, lastTangent ->
        val spline = makeSpline(lastPose?.vec(), lastTangent, f(endPosition), g(endTangent))
        val interpolator = makeTangentInterpolator(lastPose?.heading, lastTangent, spline)
        PathSegment(spline, interpolator)
    }

    /**
     * Adds a spline segment with constant heading interpolation.
     *
     * @param endPosition end position
     * @param endTangent end tangent
     */
    fun splineToConstantHeading(endPosition: Vector2d, endTangent: Double) = addSegment { f, g, lastPose, lastTangent ->
        PathSegment(
            makeSpline(lastPose?.vec(), lastTangent, f(endPosition), g(endTangent)),
            makeConstantInterpolator(lastPose?.heading)
        )
    }

    /**
     * Adds a spline segment with linear heading interpolation.
     *
     * @param endPose end pose
     * @param endTangent end tangent
     */
    fun splineToLinearHeading(endPose: Pose2d, endTangent: Double) = addSegment { f, g, lastPose, lastTangent ->
        PathSegment(
            makeSpline(lastPose?.vec(), lastTangent, f(endPose.vec()), g(endTangent)),
            makeLinearInterpolator(lastPose?.heading, g(endPose.heading))
        )
    }

    /**
     * Adds a spline segment with spline heading interpolation.
     *
     * @param endPose end pose
     * @param endTangent end tangent
     */
    fun splineToSplineHeading(endPose: Pose2d, endTangent: Double) = addSegment { f, g, lastPose, lastTangent ->
        PathSegment(
            makeSpline(lastPose?.vec(), lastTangent, f(endPose.vec()), g(endTangent)),
            makeSplineInterpolator(lastPose?.heading, g(endPose.heading))
        )
    }

    /**
     * Constructs the [Path] instance.
     */
    @JvmOverloads
    fun build(positionMap: (Vector2d) -> Vector2d = { it }, angleMap: (Double) -> Double = { it }): Path {
        val (_, segments) = cont(positionMap, angleMap)

        if (path != null && segments.isNotEmpty()) {
            val seg1 = segments[0]
            if (!(path[s!!] epsilonEqualsHeading seg1.start() &&
                    path.deriv(s) epsilonEquals seg1.startDeriv() &&
                    path.secondDeriv(s).vec() epsilonEquals seg1.startSecondDeriv().vec())
            ) {
                throw PathContinuityViolationException()
            }
        }

        segments.zip(segments.drop(1))
            .map { (seg1, seg2) ->
                if (!(seg1.end() epsilonEqualsHeading seg2.start() &&
                        seg1.endDeriv() epsilonEquals seg2.startDeriv() &&
                        seg1.endSecondDeriv().vec() epsilonEquals seg2.startSecondDeriv().vec())) {
                    throw PathContinuityViolationException()
                }
            }

        return Path(segments)
    }
}
