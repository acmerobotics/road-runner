package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.heading.*
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.epsilonEquals
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

    private fun makeConstantInterpolator(curve: ParametricCurve): ConstantInterpolator {
        val currentHeading = currentHeading ?: throw PathContinuityViolationException()

        if (segments.isNotEmpty() && !(curve.tangentAngleDeriv(0.0, 0.0) epsilonEquals 0.0)) {
            throw PathContinuityViolationException()
        }

        return ConstantInterpolator(currentHeading)
    }

    private fun makeLinearInterpolator(curve: ParametricCurve, endHeading: Double): LinearInterpolator {
        val startHeading = currentHeading ?: throw PathContinuityViolationException()

        currentHeading = endHeading
        val interpolator = LinearInterpolator(startHeading, Angle.normDelta(endHeading - startHeading))
        interpolator.init(curve)

        if (segments.isNotEmpty() && !(interpolator.startDeriv() epsilonEquals curve.tangentAngleDeriv(0.0, 0.0))) {
            throw PathContinuityViolationException()
        }

        return interpolator
    }

    private fun makeSplineInterpolator(heading: Double): SplineInterpolator {
        val interpolator = if (currentHeading == null) {
            SplineInterpolator(path!![s!!].heading, heading, path.deriv(s).heading, path.secondDeriv(s).heading, null, null)
        } else {
            SplineInterpolator(currentHeading!!, heading)
        }
        currentHeading = heading
        return interpolator
    }

    /**
     * Adds a line path segment.
     *
     * @param position end position
     * @param interpolator heading interpolator
     */
    @Deprecated("raw heading interpolators are overly verbose in high-level builders")
    fun lineTo(position: Vector2d, interpolator: HeadingInterpolator = TangentInterpolator()): PathBuilder {
        val line = makeLine(position)

        segments.add(PathSegment(line, interpolator))

        return this
    }

    /**
     * Adds a line segment with tangent heading interpolation.
     *
     * @param position end position
     */
    fun lineTo(position: Vector2d): PathBuilder {
        val line = makeLine(position)
        val interpolator = makeTangentInterpolator(line)

        segments.add(PathSegment(line, interpolator))

        return this
    }

    /**
     * Adds a line segment with constant heading interpolation.
     *
     * @param position end position
     */
    fun lineToConstantHeading(position: Vector2d): PathBuilder {
        val line = makeLine(position)
        val interpolator = makeConstantInterpolator(line)

        segments.add(PathSegment(line, interpolator))

        return this
    }

    /**
     * Adds a strafe segment (i.e., a line segment with constant heading interpolation).
     *
     * @param position end position
     */
    fun strafeTo(position: Vector2d) = lineToConstantHeading(position)

    /**
     * Adds a line segment with linear heading interpolation.
     *
     * @param position end position
     * @param heading end heading
     */
    fun lineToLinearHeading(position: Vector2d, heading: Double): PathBuilder {
        val line = makeLine(position)
        val interpolator = makeLinearInterpolator(line, heading)

        segments.add(PathSegment(line, interpolator))

        return this
    }

    /**
     * Adds a line segment with spline heading interpolation.
     *
     * @param position end position
     * @param heading end heading
     */
    fun lineToSplineHeading(position: Vector2d, heading: Double) = lineTo(position, makeSplineInterpolator(heading))

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
     * Adds a spline segment.
     *
     * @param pose end pose
     * @param interpolator heading interpolator
     */
    @Deprecated("raw heading interpolators are overly verbose in high-level builders")
    fun splineTo(pose: Pose2d, interpolator: HeadingInterpolator = TangentInterpolator()): PathBuilder {
        val spline = makeSpline(pose)

        segments.add(PathSegment(spline, interpolator))

        return this
    }

    /**
     * Adds a spline segment with tangent heading interpolation.
     *
     * @param pose end pose
     */
    fun splineTo(pose: Pose2d): PathBuilder {
        val spline = makeSpline(pose)
        val interpolator = makeTangentInterpolator(spline)

        segments.add(PathSegment(spline, interpolator))

        return this
    }

    /**
     * Adds a spline segment with constant heading interpolation.
     *
     * @param pose end pose
     */
    fun splineToConstantHeading(pose: Pose2d): PathBuilder {
        val spline = makeSpline(pose)
        val interpolator = makeConstantInterpolator(spline)

        segments.add(PathSegment(spline, interpolator))

        return this
    }

    /**
     * Adds a spline segment with linear heading interpolation.
     *
     * @param pose end pose
     * @param heading end heading
     */
    fun splineToLinearHeading(pose: Pose2d, heading: Double): PathBuilder {
        val spline = makeSpline(pose)
        val interpolator = makeLinearInterpolator(spline, heading)

        segments.add(PathSegment(spline, interpolator))

        return this
    }

    /**
     * Adds a spline segment with spline heading interpolation.
     *
     * @param pose end pose
     */
    fun splineToSplineHeading(pose: Pose2d, heading: Double) = splineTo(pose, makeSplineInterpolator(heading))

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
