package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.heading.*
import com.acmerobotics.roadrunner.util.Angle
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Easy-to-use builder for creating [Path] instances.
 *
 * @param startPose start pose
 */
class PathBuilder private constructor(
    startPose: Pose2d?,
    internal val path: Path?,
    internal val s: Double?,
    private val reversed: Boolean
) {
    @JvmOverloads
    constructor(startPose: Pose2d, reversed: Boolean = false) : this(startPose, null, null, reversed)

    @JvmOverloads
    constructor(path: Path, s: Double, reversed: Boolean = false) : this(null, path, s, reversed)

    internal var currentPose: Pose2d? = startPose

    private var segments = mutableListOf<PathSegment>()

    private fun makeLine(end: Vector2d): LineSegment {
        val start = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        return if (reversed) {
            LineSegment(end, start.vec())
        } else {
            LineSegment(start.vec(), end)
        }
    }

    private fun makeSpline(end: Pose2d): QuinticSpline {
        val (startWaypoint, endWaypoint) = if (currentPose == null) {
            val start = path!![s!!].vec()
            val startDeriv = path.internalDeriv(s).vec()
            val startSecondDeriv = path.internalSecondDeriv(s).vec()
            val derivMag = (start distTo end.vec())
            QuinticSpline.Waypoint(start, startDeriv, startSecondDeriv) to
                QuinticSpline.Waypoint(end.vec(), Vector2d(cos(end.heading) * derivMag, sin(end.heading)))
        } else {
            val derivMag = (currentPose!!.vec() distTo end.vec())
            QuinticSpline.Waypoint(currentPose!!.x, currentPose!!.y,
                derivMag * cos(currentPose!!.heading), derivMag * sin(currentPose!!.heading)) to
                QuinticSpline.Waypoint(end.x, end.y,
                    derivMag * cos(end.heading), derivMag * sin(end.heading))
        }

        return if (reversed) {
            QuinticSpline(endWaypoint, startWaypoint)
        } else {
            QuinticSpline(startWaypoint, endWaypoint)
        }
    }

    private fun makeConstantInterpolator(): ConstantInterpolator {
        // TODO
        return ConstantInterpolator(currentPose!!.heading)
    }

    private fun makeLinearInterpolator(heading: Double): LinearInterpolator {
        return if (reversed) {
            LinearInterpolator(heading, Angle.normDelta(currentPose!!.heading - heading))
        } else {
            LinearInterpolator(currentPose!!.heading, Angle.normDelta(heading - currentPose!!.heading))
        }
    }

    private fun makeSplineInterpolator(heading: Double): SplineInterpolator {
        return if (reversed) {
            SplineInterpolator(heading, currentPose!!.heading)
        } else {
            SplineInterpolator(currentPose!!.heading, heading)
        }
    }

    /**
     * Adds a line path segment.
     *
     * @param end end position
     * @param interpolator heading interpolator
     */
    @Deprecated("raw heading interpolators are no longer permitted in high-level builders")
    fun lineTo(end: Vector2d, interpolator: HeadingInterpolator = TangentInterpolator()): PathBuilder {
        val line = makeLine(end)

        segments.add(PathSegment(line, interpolator, reversed))

        val startHeading = if (currentPose == null) {
            path!![s!!].heading
        } else {
            currentPose!!.heading
        }

        currentPose = Pose2d(end, startHeading)

        return this
    }

    /**
     * Adds a strafe path segment (i.e., a line segment with constant heading).
     *
     * @param end end position
     */
    fun strafeTo(end: Vector2d) = lineTo(end, ConstantInterpolator(
            if (currentPose == null) {
                path!![s!!].heading
            } else {
                currentPose!!.heading
            }
        ))

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

        return lineTo(start.vec() + Vector2d(
            distance * cos(start.heading),
            distance * sin(start.heading)
        ))
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

        return strafeTo(start.vec() + Vector2d(
            distance * cos(start.heading + PI / 2),
            distance * sin(start.heading + PI / 2)
        ))
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
     * @param end end end
     * @param interpolator heading interpolator
     */
    @Deprecated("raw heading interpolators are no longer permitted in high-level builders")
    fun splineTo(end: Pose2d, interpolator: HeadingInterpolator = TangentInterpolator()): PathBuilder {
        val spline = makeSpline(end)

        segments.add(PathSegment(spline, interpolator, reversed))

        currentPose = end

        return this
    }

    /**
     * Adds a spline segment with tangent heading interpolation.
     *
     * @param end end end
     */
    fun splineTo(end: Pose2d) = splineTo(end, TangentInterpolator())

    /**
     * Adds a spline segment with constant heading interpolation.
     *
     * @param end end end
     */
    fun splineToConstantHeading(end: Pose2d) = splineTo(end, makeConstantInterpolator())

    /**
     * Adds a spline segment with linear heading interpolation.
     *
     * @param end end end
     */
    fun splineToLinearHeading(end: Pose2d, heading: Double) = splineTo(end, makeLinearInterpolator(heading))

    /**
     * Adds a spline segment with linear heading interpolation.
     *
     * @param end end end
     */
    fun splineToSplineHeading(end: Pose2d, heading: Double) = splineTo(end, makeSplineInterpolator(heading))

    /**
     * Constructs the [Path] instance.
     */
    fun build() = Path(segments)
}
