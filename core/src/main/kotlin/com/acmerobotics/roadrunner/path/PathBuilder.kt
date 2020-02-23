package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator
import com.acmerobotics.roadrunner.util.Angle
import kotlin.math.PI

/**
 * Exception thrown by [PathBuilder].
 */
abstract class PathBuilderException : Exception()

/**
 * Exception thrown when [PathBuilder] methods are chained illegally. This commonly arises when switching from
 * non-tangent interpolation back to tangent interpolation and when splicing paths.
 */
class PathContinuityViolationException : PathBuilderException()

/**
 * Exception thrown when empty path segments are requested.
 */
class EmptyPathSegmentException : PathBuilderException()

/**
 * Exception thrown when an empty builder (i.e., no segments) is built.
 */
class EmptyPathException : PathBuilderException()

/**
 * Easy-to-use builder for creating [Path] instances.
 *
 * @param startPose start pose
 */
class PathBuilder private constructor(
    startPose: Pose2d?,
    startHeading: Double?,
    internal val path: Path?,
    internal val s: Double?
) {
    @JvmOverloads
    constructor(startPose: Pose2d, startHeading: Double = startPose.heading) :
        this(startPose, startHeading, null, null)

    constructor(startPose: Pose2d, reversed: Boolean) :
        this(startPose, Angle.norm(startPose.heading + if (reversed) PI else 0.0))

    constructor(path: Path, s: Double) : this(null, null, path, s)

    var currentPose: Pose2d? = startPose
        private set
    var currentHeading: Double? = startHeading
        private set

    private var segments = mutableListOf<PathSegment>()

    private fun makeLine(end: Vector2d): LineSegment {
        val start = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        if (start.vec() epsilonEquals end) {
            throw EmptyPathSegmentException()
        }

        val line = LineSegment(start.vec(), end)

        currentPose = Pose2d(end, start.heading)

        return line
    }

    private fun makeSpline(end: Pose2d): QuinticSpline {
        val start = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        if (start.vec() epsilonEquals end.vec()) {
            throw EmptyPathSegmentException()
        }

        val (startWaypoint, endWaypoint) = if (currentPose == null) {
            val startDeriv = path!!.internalDeriv(s!!).vec()
            val startSecondDeriv = path.internalSecondDeriv(s).vec()
            val derivMag = (start.vec() distTo end.vec())
            QuinticSpline.Waypoint(start.vec(), startDeriv, startSecondDeriv) to
                QuinticSpline.Waypoint(end.vec(), Vector2d.polar(derivMag, end.heading))
        } else {
            val derivMag = (start.vec() distTo end.vec())
            QuinticSpline.Waypoint(start.vec(), Vector2d.polar(derivMag, start.heading)) to
                QuinticSpline.Waypoint(end.vec(), Vector2d.polar(derivMag, end.heading))
        }

        val spline = QuinticSpline(startWaypoint, endWaypoint)

        currentPose = end

        return spline
    }

    private fun makeTangentInterpolator(curve: ParametricCurve): TangentInterpolator {
        if (currentHeading == null) {
            val prevInterpolator = path!!.segment(s!!).first.interpolator
            if (prevInterpolator !is TangentInterpolator) {
                throw PathContinuityViolationException()
            }
            return TangentInterpolator(prevInterpolator.offset)
        }

        val startHeading = curve.tangentAngle(0.0, 0.0)

        val interpolator = TangentInterpolator(currentHeading!! - startHeading)
        interpolator.init(curve)
        currentHeading = interpolator.end()
        return interpolator
    }

    private fun makeConstantInterpolator(): ConstantInterpolator {
        val currentHeading = currentHeading ?: throw PathContinuityViolationException()

        return ConstantInterpolator(currentHeading)
    }

    private fun makeLinearInterpolator(endHeading: Double): LinearInterpolator {
        val startHeading = currentHeading ?: throw PathContinuityViolationException()

        currentHeading = endHeading

        return LinearInterpolator(startHeading, Angle.normDelta(endHeading - startHeading))
    }

    private fun makeSplineInterpolator(endHeading: Double): SplineInterpolator {
        val interpolator = if (currentHeading == null) {
            SplineInterpolator(path!![s!!].heading, endHeading, path.deriv(s).heading, path.secondDeriv(s).heading, null, null)
        } else {
            SplineInterpolator(currentHeading!!, endHeading)
        }
        currentHeading = endHeading
        return interpolator
    }

    private fun addSegment(segment: PathSegment): PathBuilder {
        if (segments.isNotEmpty()) {
            val lastSegment = segments.last()
            if (!(lastSegment.end() epsilonEqualsHeading segment.start() &&
                lastSegment.endDeriv() epsilonEquals segment.startDeriv() &&
                lastSegment.endSecondDeriv().vec() epsilonEquals segment.startSecondDeriv().vec())) {
                throw PathContinuityViolationException()
            }
        } else if (currentPose == null) {
            if (!(path!![s!!] epsilonEqualsHeading segment.start() &&
                path.deriv(s) epsilonEquals segment.startDeriv() &&
                path.secondDeriv(s).vec() epsilonEquals segment.startSecondDeriv().vec())) {
                throw PathContinuityViolationException()
            }
        }

        segments.add(segment)

        return this
    }

    /**
     * Adds a line segment with tangent heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineTo(endPosition: Vector2d): PathBuilder {
        val line = makeLine(endPosition)
        val interpolator = makeTangentInterpolator(line)

        addSegment(PathSegment(line, interpolator))

        return this
    }

    /**
     * Adds a line segment with constant heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineToConstantHeading(endPosition: Vector2d) =
        addSegment(PathSegment(makeLine(endPosition), makeConstantInterpolator()))

    /**
     * Adds a strafe segment (i.e., a line segment with constant heading interpolation).
     *
     * @param endPosition end position
     */
    fun strafeTo(endPosition: Vector2d) = lineToConstantHeading(endPosition)

    /**
     * Adds a line segment with linear heading interpolation.
     *
     * @param endPosition end position
     * @param heading end heading
     */
    fun lineToLinearHeading(endPosition: Vector2d, heading: Double) =
        addSegment(PathSegment(makeLine(endPosition), makeLinearInterpolator(heading)))

    /**
     * Adds a line segment with spline heading interpolation.
     *
     * @param endPosition end position
     * @param heading end heading
     */
    fun lineToSplineHeading(endPosition: Vector2d, heading: Double) =
        addSegment(PathSegment(makeLine(endPosition), makeSplineInterpolator(heading)))

    /**
     * Adds a line straight forward.
     *
     * @param distance distance to travel forward
     */
    fun forward(distance: Double): PathBuilder {
        val start = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        return lineTo(start.vec() + Vector2d.polar(distance, start.heading))
    }

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
    fun strafeLeft(distance: Double): PathBuilder {
        val start = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        return strafeTo(start.vec() + Vector2d.polar(distance, start.heading + PI / 2))
    }

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
     * @param endPose end pose
     */
    fun splineTo(endPose: Pose2d): PathBuilder {
        val spline = makeSpline(endPose)
        val interpolator = makeTangentInterpolator(spline)

        return addSegment(PathSegment(spline, interpolator))
    }

    /**
     * Adds a spline segment with constant heading interpolation.
     *
     * @param endPose end pose
     */
    fun splineToConstantHeading(endPose: Pose2d) =
        addSegment(PathSegment(makeSpline(endPose), makeConstantInterpolator()))

    /**
     * Adds a spline segment with linear heading interpolation.
     *
     * @param endPose end pose
     * @param endHeading end heading
     */
    fun splineToLinearHeading(endPose: Pose2d, endHeading: Double) =
        addSegment(PathSegment(makeSpline(endPose), makeLinearInterpolator(endHeading)))

    /**
     * Adds a spline segment with spline heading interpolation.
     *
     * @param endPose end pose
     */
    fun splineToSplineHeading(endPose: Pose2d, endHeading: Double) =
        addSegment(PathSegment(makeSpline(endPose), makeSplineInterpolator(endHeading)))

    /**
     * Constructs the [Path] instance.
     */
    fun build(): Path {
        if (segments.size < 1) {
            throw EmptyPathException()
        }
        return Path(segments)
    }
}
