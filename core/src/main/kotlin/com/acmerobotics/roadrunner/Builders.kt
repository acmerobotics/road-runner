package com.acmerobotics.roadrunner

import kotlin.math.abs

/**
 * @usesMathJax
 *
 * Persistent builder for a [CompositePositionPath] with an [Arclength] parameter that guarantees \(C^2\) continuity.
 */
class PositionPathBuilder private constructor(
    // invariants:
    // - segments satisfy continuity guarantees
    // - last segment ends with nextBeginPos, nextBeginTangent if it exists
    private val segments: List<PositionPath<Arclength>>,
    private val nextBeginPos: Vector2d,
    private val nextBeginTangent: Rotation2d,
    private val eps: Double,
) {

    /**
     * @usesMathJax
     *
     * Exception thrown when a path segment is added that doesn't maintain \(C^2\) position continuity.
     */
    class PathContinuityException : RuntimeException()

    constructor(
        beginPos: Vector2d,
        beginTangent: Rotation2d,
        eps: Double,
    ) : this(emptyList(), beginPos, beginTangent, eps)

    constructor(
        beginPos: Vector2d,
        beginTangent: Double,
        eps: Double,
    ) : this(emptyList(), beginPos, Rotation2d.exp(beginTangent), eps)

    private fun addSegment(p: PositionPath<Arclength>): PositionPathBuilder {
        val begin = p.begin(2)
        val beginPos = begin.value()
        val beginTangent = begin.drop(1).value().angleCast()

        if (abs(nextBeginPos.x - beginPos.x) > eps ||
            abs(nextBeginPos.y - beginPos.y) > eps ||
            abs(nextBeginTangent - beginTangent) > 1e-6
        ) {
            throw PathContinuityException()
        }

        val end = p.end(2)
        return PositionPathBuilder(
            segments + listOf(p),
            end.value(),
            end.drop(1).value().angleCast(),
            eps
        )
    }

    /**
     * Adds a line segment that goes forward distance [dist].
     */
    fun forward(dist: Double) =
        addSegment(Line(nextBeginPos, nextBeginPos + nextBeginTangent.vec() * dist))

    /**
     * @usesMathJax
     *
     * Adds a line segment that goes forward to \(x\)-coordinate [posX].
     */
    fun lineToX(posX: Double) =
        addSegment(
            Line(
                nextBeginPos,
                Vector2d(
                    posX,
                    (posX - nextBeginPos.x) / nextBeginTangent.real * nextBeginTangent.imag + nextBeginPos.y
                )
            )
        )

    /**
     * @usesMathJax
     *
     * Adds a line segment that goes forward to \(y\)-coordinate [posY].
     */
    fun lineToY(posY: Double) = addSegment(
        Line(
            nextBeginPos,
            Vector2d(
                (posY - nextBeginPos.y) / nextBeginTangent.imag * nextBeginTangent.real + nextBeginPos.x, posY,
            )
        )
    )

    /**
     * Adds a spline segment to position [pos] with tangent [tangent].
     */
    fun splineTo(pos: Vector2d, tangent: Rotation2d): PositionPathBuilder {
        val dist = (pos - nextBeginPos).norm()

        if (dist < eps) {
            return this
        }

        // note: First derivatives will be normalized by arc length reparam, so the magnitudes need not match at knots.
        val beginDeriv = nextBeginTangent.vec() * dist
        val endDeriv = tangent.vec() * dist

        val spline = ArclengthReparamCurve2(
            QuinticSpline2(
                QuinticSpline1(
                    DualNum(doubleArrayOf(nextBeginPos.x, beginDeriv.x, 0.0)),
                    DualNum(doubleArrayOf(pos.x, endDeriv.x, 0.0))
                ),
                QuinticSpline1(
                    DualNum(doubleArrayOf(nextBeginPos.y, beginDeriv.y, 0.0)),
                    DualNum(doubleArrayOf(pos.y, endDeriv.y, 0.0)),
                )
            ),
            eps
        )

        return addSegment(spline)
    }

    /**
     * Adds a spline segment to position [pos] with tangent [tangent].
     */
    fun splineTo(pos: Vector2d, tangent: Double) = splineTo(pos, Rotation2d.exp(tangent))

    fun build() = CompositePositionPath(segments)
}

/**
 * @usesMathJax
 *
 * Persistent builder for a [CompositePosePath] that guarantees \(C^1\) heading continuity given an
 * [Arclength]-parameterized [PositionPath] with \(C^2\) continuity.
 *
 * Throws a [RotationContinuityException] exception when a rotation segment is added that doesn't maintain \(C^1\)
 * heading continuity. To avoid this, keep one or more [splineUntil] calls between every call to [tangentUntil],
 * [constantUntil], or [linearUntil]. This discipline is enforced by the interface of [SafePosePathBuilder], and that
 * builder is recommended over this one.
 */
class PosePathBuilder private constructor(
    // invariants:
    // - posPath is C2-continuous
    // - state segments satisfy continuity guarantees
    // - state encodes heading for [0.0, beginDisp)
    private val posPath: PositionPath<Arclength>,
    private val beginDisp: Double,
    private val state: State,
) {
    constructor(path: PositionPath<Arclength>, beginHeading: Rotation2d) :
        this(path, 0.0, Lazy({ emptyList() }, beginHeading))

    constructor(path: PositionPath<Arclength>, beginHeading: Double) :
        this(path, 0.0, Lazy({ emptyList() }, Rotation2d.exp(beginHeading)))

    private sealed interface State {
        val endHeading: Rotation2d
    }

    private class Eager(
        val segments: List<PosePath>,
        val endHeadingDual: Rotation2dDual<Arclength>
    ) : State {
        override val endHeading = endHeadingDual.value()
    }

    private class Lazy(
        val makePaths: (Rotation2dDual<Arclength>) -> List<PosePath>,
        override val endHeading: Rotation2d
    ) : State

    /**
     * @usesMathJax
     *
     * Exception thrown when a rotation segment is added that doesn't maintain \(C^1\) heading continuity.
     */
    class RotationContinuityException : RuntimeException()

    private fun addEagerPosePath(disp: Double, segment: PosePath): PosePathBuilder {
        require(disp > beginDisp)

        val beginHeadingDual = segment.begin(3).rot

        return PosePathBuilder(
            posPath, disp,
            Eager(
                when (state) {
                    is Eager -> {
                        fun <Param> DualNum<Param>.epsilonEquals(n: DualNum<Param>) =
                            values().zip(n.values()).all { (x, y) -> abs(x - y) < 1e-6 }

                        fun <Param> Rotation2dDual<Param>.epsilonEquals(r: Rotation2dDual<Param>) =
                            real.epsilonEquals(r.real) && imag.epsilonEquals(r.imag)

                        if (!state.endHeadingDual.epsilonEquals(beginHeadingDual)) {
                            throw RotationContinuityException()
                        }

                        state.segments
                    }
                    is Lazy -> state.makePaths(beginHeadingDual)
                } + listOf(segment),
                segment.end(3).rot
            )
        )
    }

    private fun viewUntil(disp: Double) =
        PositionPathView(posPath, beginDisp, disp - beginDisp)

    /**
     * Fills in tangent headings until displacement [disp].
     */
    fun tangentUntil(disp: Double) = addEagerPosePath(
        disp,
        TangentPath(
            viewUntil(disp),
            state.endHeading - posPath[disp, 2].drop(1).value().angleCast()
        )
    )

    /**
     * Fills in constant headings until displacement [disp].
     */
    fun constantUntil(disp: Double) = addEagerPosePath(
        disp,
        HeadingPosePath(
            viewUntil(disp),
            ConstantHeadingPath(state.endHeading, disp - beginDisp),
        )
    )

    /**
     * Fills in headings interpolated linearly to heading [heading] at displacement [disp].
     */
    fun linearUntil(disp: Double, heading: Rotation2d) = addEagerPosePath(
        disp,
        HeadingPosePath(
            viewUntil(disp),
            LinearHeadingPath(state.endHeading, heading - state.endHeading, disp - beginDisp)
        )
    )

    /**
     * Fills in headings interpolated linearly to heading [heading] at displacement [disp].
     */
    fun linearUntil(disp: Double, heading: Double) = linearUntil(disp, Rotation2d.exp(heading))

    /**
     * @usesMathJax
     *
     * Fills in headings interpolated with a spline to heading [heading] at displacement [disp].
     *
     * Flexibility in the choice of spline endpoint derivatives allows [splineUntil] to both precede and succeed any
     * other heading segment. And in fact the heading at both knots will be \(C^2\)-continuous.
     */
    fun splineUntil(disp: Double, heading: Rotation2d): PosePathBuilder {
        require(disp > beginDisp)

        return PosePathBuilder(
            posPath, disp,
            Lazy(
                when (state) {
                    is Eager -> {
                        {
                            state.segments + listOf(
                                HeadingPosePath(
                                    viewUntil(disp),
                                    SplineHeadingPath(state.endHeadingDual, it, disp - beginDisp),
                                )
                            )
                        }
                    }
                    is Lazy -> {
                        {
                            val beginTangent = posPath[beginDisp, 4].drop(1).angleCast()
                            val beginHeading = beginTangent.withRot(state.endHeading)

                            state.makePaths(beginHeading) + listOf(
                                HeadingPosePath(
                                    viewUntil(disp),
                                    SplineHeadingPath(beginHeading, it, disp - beginDisp)
                                )
                            )
                        }
                    }
                },
                heading
            )
        )
    }

    /**
     * @usesMathJax
     *
     * Fills in headings interpolated with a spline to heading [heading] at displacement [disp].
     *
     * Flexibility in the choice of spline endpoint derivatives allows [splineUntil] to both precede and succeed any
     * other heading segment. And in fact the heading at both knots will be \(C^2\)-continuous.
     */
    fun splineUntil(disp: Double, heading: Double) = splineUntil(disp, Rotation2d.exp(heading))

    fun tangentUntilEnd() = tangentUntil(posPath.length()).build()
    fun constantUntilEnd() = constantUntil(posPath.length()).build()
    fun linearUntilEnd(heading: Rotation2d) = linearUntil(posPath.length(), heading).build()
    fun linearUntilEnd(heading: Double) = linearUntilEnd(Rotation2d.exp(heading))
    fun splineUntilEnd(heading: Rotation2d) = splineUntil(posPath.length(), heading).build()
    fun splineUntilEnd(heading: Double) = splineUntilEnd(Rotation2d.exp(heading))

    internal fun build(): CompositePosePath {
        require(beginDisp == posPath.length())

        return CompositePosePath(
            when (state) {
                is Eager -> state.segments
                is Lazy -> {
                    val endTangent = posPath[beginDisp, 4].drop(1).angleCast()
                    val endHeading = endTangent.withRot(state.endHeading)

                    state.makePaths(endHeading)
                }
            }
        )
    }
}

/**
 * Wrapper for [PosePathBuilder] that provides the same guarantees without throwing
 * [PosePathBuilder.RotationContinuityException].
 *
 * For method-by-method documentation, see the identical methods on [PosePathBuilder].
 */
class SafePosePathBuilder internal constructor(private val posePathBuilder: PosePathBuilder) {
    constructor(path: PositionPath<Arclength>, beginHeading: Rotation2d) :
        this(PosePathBuilder(path, beginHeading))
    constructor(path: PositionPath<Arclength>, beginHeading: Double) :
        this(PosePathBuilder(path, beginHeading))

    fun tangentUntil(disp: Double) =
        RestrictedPosePathBuilder(posePathBuilder.tangentUntil(disp))
    fun constantUntil(disp: Double) =
        RestrictedPosePathBuilder(posePathBuilder.constantUntil(disp))
    fun linearUntil(disp: Double, heading: Rotation2d) =
        RestrictedPosePathBuilder(posePathBuilder.linearUntil(disp, heading))
    fun linearUntil(disp: Double, heading: Double) =
        RestrictedPosePathBuilder(posePathBuilder.linearUntil(disp, heading))

    fun splineUntil(disp: Double, heading: Rotation2d) =
        SafePosePathBuilder(posePathBuilder.splineUntil(disp, heading))
    fun splineUntil(disp: Double, heading: Double) =
        SafePosePathBuilder(posePathBuilder.splineUntil(disp, heading))

    fun tangentUntilEnd() = posePathBuilder.tangentUntilEnd()
    fun constantUntilEnd() = posePathBuilder.constantUntilEnd()
    fun linearUntilEnd(heading: Rotation2d) = posePathBuilder.linearUntilEnd(heading)
    fun linearUntilEnd(heading: Double) = posePathBuilder.linearUntilEnd(heading)
    fun splineUntilEnd(heading: Rotation2d) = posePathBuilder.splineUntilEnd(heading)
    fun splineUntilEnd(heading: Double) = posePathBuilder.splineUntilEnd(heading)
}

/**
 * Do not instantiate directly. See [SafePosePathBuilder].
 */
class RestrictedPosePathBuilder internal constructor(private val posePathBuilder: PosePathBuilder) {
    fun splineUntil(disp: Double, heading: Rotation2d) =
        SafePosePathBuilder(posePathBuilder.splineUntil(disp, heading))
    fun splineUntil(disp: Double, heading: Double) =
        SafePosePathBuilder(posePathBuilder.splineUntil(disp, heading))

    fun splineUntilEnd(heading: Rotation2d) = posePathBuilder.splineUntilEnd(heading)
    fun splineUntilEnd(heading: Double) = posePathBuilder.splineUntilEnd(heading)
}

class PathBuilder private constructor(
    private val posPathBuilder: PositionPathBuilder,
    private val posePathBuilder: (CompositePositionPath<Arclength>, Int) -> PosePathBuilder
) {
    constructor(beginPose: Pose2d, beginTangent: Rotation2d, eps: Double) :
        this(
            PositionPathBuilder(beginPose.trans, beginTangent, eps),
            { path, _ -> PosePathBuilder(path, beginPose.rot) }
        )
    constructor(beginPose: Pose2d, beginTangent: Double, eps: Double) :
        this(beginPose, Rotation2d.exp(beginTangent), eps)

    private fun addHeadingSegment(f: PosePathBuilder.(Double) -> PosePathBuilder): (
        CompositePositionPath<Arclength>,
        Int
    ) -> PosePathBuilder = { path, i ->
        posePathBuilder(path, i - 1).f(path.offsets[i])
    }

    fun forward(dist: Double) = PathBuilder(posPathBuilder.forward(dist), addHeadingSegment { this.tangentUntil(it) })

    fun forwardConstantHeading(dist: Double) =
        PathBuilder(posPathBuilder.forward(dist), addHeadingSegment { this.constantUntil(it) })

    fun forwardLinearHeading(dist: Double, heading: Rotation2d) =
        PathBuilder(posPathBuilder.forward(dist), addHeadingSegment { this.linearUntil(it, heading) })
    fun forwardLinearHeading(dist: Double, heading: Double) = forwardLinearHeading(dist, Rotation2d.exp(heading))

    fun forwardSplineHeading(dist: Double, heading: Rotation2d) =
        PathBuilder(posPathBuilder.forward(dist), addHeadingSegment { this.splineUntil(it, heading) })
    fun forwardSplineHeading(dist: Double, heading: Double) = forwardSplineHeading(dist, Rotation2d.exp(heading))

    fun lineToX(posX: Double) = PathBuilder(posPathBuilder.lineToX(posX), addHeadingSegment { this.tangentUntil(it) })

    fun lineToXConstantHeading(posX: Double) =
        PathBuilder(posPathBuilder.lineToX(posX), addHeadingSegment { this.constantUntil(it) })

    fun lineToXLinearHeading(posX: Double, heading: Rotation2d) =
        PathBuilder(posPathBuilder.lineToX(posX), addHeadingSegment { this.linearUntil(it, heading) })
    fun lineToXLinearHeading(posX: Double, heading: Double) = lineToXLinearHeading(posX, Rotation2d.exp(heading))

    fun lineToXSplineHeading(posX: Double, heading: Rotation2d) =
        PathBuilder(posPathBuilder.lineToX(posX), addHeadingSegment { this.splineUntil(it, heading) })
    fun lineToXSplineHeading(posX: Double, heading: Double) = lineToXSplineHeading(posX, Rotation2d.exp(heading))

    fun lineToY(posY: Double) = PathBuilder(posPathBuilder.lineToY(posY), addHeadingSegment { this.tangentUntil(it) })

    fun lineToYConstantHeading(posY: Double) =
        PathBuilder(posPathBuilder.lineToY(posY), addHeadingSegment { this.constantUntil(it) })

    fun lineToYLinearHeading(posY: Double, heading: Rotation2d) =
        PathBuilder(posPathBuilder.lineToY(posY), addHeadingSegment { this.linearUntil(it, heading) })
    fun lineToYLinearHeading(posY: Double, heading: Double) = lineToYLinearHeading(posY, Rotation2d.exp(heading))

    fun lineToYSplineHeading(posY: Double, heading: Rotation2d) =
        PathBuilder(posPathBuilder.lineToY(posY), addHeadingSegment { this.splineUntil(it, heading) })
    fun lineToYSplineHeading(posY: Double, heading: Double) = lineToYSplineHeading(posY, Rotation2d.exp(heading))

    fun splineTo(pos: Vector2d, tangent: Rotation2d) =
        PathBuilder(posPathBuilder.splineTo(pos, tangent), addHeadingSegment { this.tangentUntil(it) })
    fun splineTo(pos: Vector2d, tangent: Double) = splineTo(pos, Rotation2d.exp(tangent))

    fun splineToConstantHeading(pos: Vector2d, tangent: Rotation2d) =
        PathBuilder(posPathBuilder.splineTo(pos, tangent), addHeadingSegment { this.constantUntil(it) })
    fun splineToConstantHeading(pos: Vector2d, tangent: Double) =
        splineToConstantHeading(pos, Rotation2d.exp(tangent))

    fun splineToLinearHeading(pose: Pose2d, tangent: Rotation2d) = PathBuilder(
        posPathBuilder.splineTo(pose.trans, tangent), addHeadingSegment { this.linearUntil(it, pose.rot) }
    )
    fun splineToLinearHeading(pose: Pose2d, tangent: Double) = splineToLinearHeading(pose, Rotation2d.exp(tangent))

    fun splineToSplineHeading(pose: Pose2d, tangent: Rotation2d) = PathBuilder(
        posPathBuilder.splineTo(pose.trans, tangent), addHeadingSegment { this.splineUntil(it, pose.rot) }
    )
    fun splineToSplineHeading(pose: Pose2d, tangent: Double) = splineToSplineHeading(pose, Rotation2d.exp(tangent))

    fun build(): CompositePosePath {
        val posPath = posPathBuilder.build()
        return posePathBuilder(posPath, posPath.offsets.lastIndex).build()
    }
}

class SafePathBuilder internal constructor(private val pathBuilder: PathBuilder) {
    constructor(beginPose: Pose2d, beginTangent: Rotation2d, eps: Double) :
        this(PathBuilder(beginPose, beginTangent, eps))
    constructor(beginPose: Pose2d, beginTangent: Double, eps: Double) :
        this(beginPose, Rotation2d.exp(beginTangent), eps)

    fun forward(dist: Double) = TangentPathBuilder(pathBuilder.forward(dist))
    fun forwardConstantHeading(dist: Double) = ConstantPathBuilder(pathBuilder.forwardConstantHeading(dist))
    fun forwardLinearHeading(dist: Double, heading: Rotation2d) =
        RestrictedPathBuilder(pathBuilder.forwardLinearHeading(dist, heading))
    fun forwardLinearHeading(dist: Double, heading: Double) = forwardLinearHeading(dist, Rotation2d.exp(heading))
    fun forwardSplineHeading(dist: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.forwardSplineHeading(dist, heading))
    fun forwardSplineHeading(dist: Double, heading: Double) = forwardSplineHeading(dist, Rotation2d.exp(heading))

    fun lineToX(posX: Double) = TangentPathBuilder(pathBuilder.lineToX(posX))
    fun lineToXConstantHeading(posX: Double) = ConstantPathBuilder(pathBuilder.lineToXConstantHeading(posX))
    fun lineToXLinearHeading(posX: Double, heading: Rotation2d) =
        RestrictedPathBuilder(pathBuilder.lineToXLinearHeading(posX, heading))
    fun lineToXLinearHeading(posX: Double, heading: Double) = lineToXLinearHeading(posX, Rotation2d.exp(heading))
    fun lineToXSplineHeading(posX: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToXSplineHeading(posX, heading))
    fun lineToXSplineHeading(posX: Double, heading: Double) = lineToXSplineHeading(posX, Rotation2d.exp(heading))

    fun lineToY(posY: Double) = TangentPathBuilder(pathBuilder.lineToY(posY))
    fun lineToYConstantHeading(posY: Double) = ConstantPathBuilder(pathBuilder.lineToXConstantHeading(posY))
    fun lineToYLinearHeading(posY: Double, heading: Rotation2d) =
        RestrictedPathBuilder(pathBuilder.lineToYLinearHeading(posY, heading))
    fun lineToYLinearHeading(posY: Double, heading: Double) = lineToYLinearHeading(posY, Rotation2d.exp(heading))
    fun lineToYSplineHeading(posY: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToYSplineHeading(posY, heading))
    fun lineToYSplineHeading(posY: Double, heading: Double) = lineToYSplineHeading(posY, Rotation2d.exp(heading))

    fun splineTo(pos: Vector2d, tangent: Rotation2d) = TangentPathBuilder(pathBuilder.splineTo(pos, tangent))
    fun splineTo(pos: Vector2d, tangent: Double) = splineTo(pos, Rotation2d.exp(tangent))
    fun splineToConstantHeading(pos: Vector2d, tangent: Rotation2d) =
        ConstantPathBuilder(pathBuilder.splineToConstantHeading(pos, tangent))
    fun splineToConstantHeading(pos: Vector2d, tangent: Double) =
        splineToConstantHeading(pos, Rotation2d.exp(tangent))
    fun splineToLinearHeading(pose: Pose2d, tangent: Rotation2d) =
        RestrictedPathBuilder(pathBuilder.splineToLinearHeading(pose, tangent))
    fun splineToLinearHeading(pose: Pose2d, tangent: Double) = splineToLinearHeading(pose, Rotation2d.exp(tangent))
    fun splineToSplineHeading(pose: Pose2d, tangent: Rotation2d) =
        SafePathBuilder(pathBuilder.splineToSplineHeading(pose, tangent))
    fun splineToSplineHeading(pose: Pose2d, tangent: Double) = splineToSplineHeading(pose, Rotation2d.exp(tangent))

    fun build() = pathBuilder.build()
}

class TangentPathBuilder internal constructor(private val pathBuilder: PathBuilder) {
    fun forward(dist: Double) = TangentPathBuilder(pathBuilder.forward(dist))
    fun forwardSplineHeading(dist: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.forwardSplineHeading(dist, heading))
    fun forwardSplineHeading(dist: Double, heading: Double) = forwardSplineHeading(dist, Rotation2d.exp(heading))

    fun lineToX(posX: Double) = TangentPathBuilder(pathBuilder.lineToX(posX))
    fun lineToXSplineHeading(posX: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToXSplineHeading(posX, heading))
    fun lineToXSplineHeading(posX: Double, heading: Double) = lineToXSplineHeading(posX, Rotation2d.exp(heading))

    fun lineToY(posY: Double) = TangentPathBuilder(pathBuilder.lineToY(posY))
    fun lineToYSplineHeading(posY: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToYSplineHeading(posY, heading))
    fun lineToYSplineHeading(posY: Double, heading: Double) = lineToYSplineHeading(posY, Rotation2d.exp(heading))

    fun splineTo(pos: Vector2d, tangent: Rotation2d) = TangentPathBuilder(pathBuilder.splineTo(pos, tangent))
    fun splineTo(pos: Vector2d, tangent: Double) = splineTo(pos, Rotation2d.exp(tangent))
    fun splineToSplineHeading(pose: Pose2d, tangent: Rotation2d) =
        SafePathBuilder(pathBuilder.splineToSplineHeading(pose, tangent))
    fun splineToSplineHeading(pose: Pose2d, tangent: Double) = splineToSplineHeading(pose, Rotation2d.exp(tangent))

    fun build() = pathBuilder.build()
}

class ConstantPathBuilder internal constructor(private val pathBuilder: PathBuilder) {
    fun forwardConstantHeading(dist: Double) = ConstantPathBuilder(pathBuilder.forwardConstantHeading(dist))
    fun forwardSplineHeading(dist: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.forwardSplineHeading(dist, heading))
    fun forwardSplineHeading(dist: Double, heading: Double) = forwardSplineHeading(dist, Rotation2d.exp(heading))

    fun lineToXConstantHeading(posX: Double) = ConstantPathBuilder(pathBuilder.lineToXConstantHeading(posX))
    fun lineToXSplineHeading(posX: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToXSplineHeading(posX, heading))
    fun lineToXSplineHeading(posX: Double, heading: Double) = lineToXSplineHeading(posX, Rotation2d.exp(heading))

    fun lineToYConstantHeading(posY: Double) = ConstantPathBuilder(pathBuilder.lineToXConstantHeading(posY))
    fun lineToYSplineHeading(posY: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToYSplineHeading(posY, heading))
    fun lineToYSplineHeading(posY: Double, heading: Double) = lineToYSplineHeading(posY, Rotation2d.exp(heading))

    fun splineToConstantHeading(pos: Vector2d, tangent: Rotation2d) =
        ConstantPathBuilder(pathBuilder.splineToConstantHeading(pos, tangent))
    fun splineToConstantHeading(pos: Vector2d, tangent: Double) =
        splineToConstantHeading(pos, Rotation2d.exp(tangent))
    fun splineToSplineHeading(pose: Pose2d, tangent: Rotation2d) =
        SafePathBuilder(pathBuilder.splineToSplineHeading(pose, tangent))
    fun splineToSplineHeading(pose: Pose2d, tangent: Double) = splineToSplineHeading(pose, Rotation2d.exp(tangent))

    fun build() = pathBuilder.build()
}

class RestrictedPathBuilder internal constructor(private val pathBuilder: PathBuilder) {
    fun forwardSplineHeading(dist: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.forwardSplineHeading(dist, heading))
    fun forwardSplineHeading(dist: Double, heading: Double) = forwardSplineHeading(dist, Rotation2d.exp(heading))

    fun lineToXSplineHeading(posX: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToXSplineHeading(posX, heading))
    fun lineToXSplineHeading(posX: Double, heading: Double) = lineToXSplineHeading(posX, Rotation2d.exp(heading))

    fun lineToYSplineHeading(posY: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToYSplineHeading(posY, heading))
    fun lineToYSplineHeading(posY: Double, heading: Double) = lineToYSplineHeading(posY, Rotation2d.exp(heading))

    fun splineToSplineHeading(pose: Pose2d, tangent: Rotation2d) =
        SafePathBuilder(pathBuilder.splineToSplineHeading(pose, tangent))
    fun splineToSplineHeading(pose: Pose2d, tangent: Double) = splineToSplineHeading(pose, Rotation2d.exp(tangent))

    fun build() = pathBuilder.build()
}

fun interface PoseMapper {
    fun map(pose: Pose2d): Pose2d
}

class TrajectoryBuilder private constructor(
    private val pathBuilder: PathBuilder,
    private val beginEndVel: Double,
    private val baseVelConstraint: VelConstraint,
    private val baseAccelConstraint: AccelConstraint,
    private val resolution: Double,
    private val poseMapper: PoseMapper,
    private val velConstraints: List<VelConstraint>,
    private val accelConstraints: List<AccelConstraint>,
) {
    @JvmOverloads
    constructor(
        beginPose: Pose2d,
        beginTangent: Rotation2d,
        eps: Double,
        beginEndVel: Double,
        baseVelConstraint: VelConstraint,
        baseAccelConstraint: AccelConstraint,
        resolution: Double,
        poseMapper: PoseMapper = PoseMapper { it }
    ) :
        this(
            PathBuilder(beginPose, beginTangent, eps),
            beginEndVel, baseVelConstraint, baseAccelConstraint, resolution,
            poseMapper, emptyList(), emptyList()
        )

    private fun add(
        newPathBuilder: PathBuilder,
        velConstraintOverride: VelConstraint?,
        accelConstraintOverride: AccelConstraint?
    ) =
        TrajectoryBuilder(
            newPathBuilder, beginEndVel, baseVelConstraint, baseAccelConstraint, resolution, poseMapper,
            velConstraints + listOf(velConstraintOverride ?: baseVelConstraint),
            accelConstraints + listOf(accelConstraintOverride ?: baseAccelConstraint)
        )

    @JvmOverloads
    fun forward(
        dist: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.forward(dist), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun forwardConstantHeading(
        dist: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.forwardConstantHeading(dist), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun forwardLinearHeading(
        dist: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.forwardLinearHeading(dist, heading), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun forwardLinearHeading(
        dist: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.forwardLinearHeading(dist, heading), velConstraintOverride, accelConstraintOverride)

    fun forwardSplineHeading(
        dist: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.forwardSplineHeading(dist, heading), velConstraintOverride, accelConstraintOverride)
    fun forwardSplineHeading(
        dist: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.forwardSplineHeading(dist, heading), velConstraintOverride, accelConstraintOverride)

    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToX(posX), velConstraintOverride, accelConstraintOverride)

    fun lineToXConstantHeading(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXConstantHeading(posX), velConstraintOverride, accelConstraintOverride)

    fun lineToXLinearHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXLinearHeading(posX, heading), velConstraintOverride, accelConstraintOverride)
    fun lineToXLinearHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXLinearHeading(posX, heading), velConstraintOverride, accelConstraintOverride)

    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXSplineHeading(posX, heading), velConstraintOverride, accelConstraintOverride)
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXSplineHeading(posX, heading), velConstraintOverride, accelConstraintOverride)

    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToY(posY), velConstraintOverride, accelConstraintOverride)

    fun lineToYConstantHeading(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYConstantHeading(posY), velConstraintOverride, accelConstraintOverride)

    fun lineToYLinearHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYLinearHeading(posY, heading), velConstraintOverride, accelConstraintOverride)
    fun lineToYLinearHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYLinearHeading(posY, heading), velConstraintOverride, accelConstraintOverride)

    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYSplineHeading(posY, heading), velConstraintOverride, accelConstraintOverride)
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYSplineHeading(posY, heading), velConstraintOverride, accelConstraintOverride)

    fun splineTo(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineTo(pos, tangent), velConstraintOverride, accelConstraintOverride)
    fun splineTo(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineTo(pos, tangent), velConstraintOverride, accelConstraintOverride)

    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToConstantHeading(pos, tangent), velConstraintOverride, accelConstraintOverride)
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToConstantHeading(pos, tangent), velConstraintOverride, accelConstraintOverride)

    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToLinearHeading(pose, tangent), velConstraintOverride, accelConstraintOverride)
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToLinearHeading(pose, tangent), velConstraintOverride, accelConstraintOverride)

    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToSplineHeading(pose, tangent), velConstraintOverride, accelConstraintOverride)
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToSplineHeading(pose, tangent), velConstraintOverride, accelConstraintOverride)

    fun build(): Trajectory {
        val path = pathBuilder.build()
        return Trajectory(
            object : PosePath {
                override fun length() = path.length
                override fun get(s: Double, n: Int): Pose2dDual<Arclength> {
                    val pose = path[s, n]
                    return pose.withTransform(poseMapper.map(pose.value()))
                }
            },
            profile(
                path, beginEndVel,
                CompositeVelConstraint(velConstraints, path.offsets.drop(1)),
                CompositeAccelConstraint(accelConstraints, path.offsets.drop(1)),
                resolution,
            ),
            path.offsets
        )
    }
}

class SafeTrajectoryBuilder internal constructor(private val trajBuilder: TrajectoryBuilder) {
    @JvmOverloads
    constructor(
        beginPose: Pose2d,
        beginTangent: Rotation2d,
        eps: Double,
        beginEndVel: Double,
        baseVelConstraint: VelConstraint,
        baseAccelConstraint: AccelConstraint,
        resolution: Double,
        poseMapper: PoseMapper = PoseMapper { it }
    ) :
        this(
            TrajectoryBuilder(
                beginPose, beginTangent, eps,
                beginEndVel, baseVelConstraint, baseAccelConstraint, resolution,
                poseMapper,
            )
        )

    @JvmOverloads
    fun forward(
        dist: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TangentTrajectoryBuilder(trajBuilder.forward(dist, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun forwardConstantHeading(
        dist: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = ConstantTrajectoryBuilder(
        trajBuilder.forwardConstantHeading(
            dist, velConstraintOverride, accelConstraintOverride
        )
    )
    @JvmOverloads
    fun forwardLinearHeading(
        dist: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        RestrictedTrajectoryBuilder(
            trajBuilder.forwardLinearHeading(
                dist, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun forwardLinearHeading(
        dist: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = forwardLinearHeading(dist, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun forwardSplineHeading(
        dist: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.forwardSplineHeading(
                dist, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun forwardSplineHeading(
        dist: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = forwardSplineHeading(dist, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TangentTrajectoryBuilder(trajBuilder.lineToX(posX, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToXConstantHeading(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = ConstantTrajectoryBuilder(
        trajBuilder.lineToXConstantHeading(
            posX, velConstraintOverride, accelConstraintOverride
        )
    )
    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        RestrictedTrajectoryBuilder(
            trajBuilder.lineToXLinearHeading(
                posX, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToXLinearHeading(posX, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToXSplineHeading(
                posX, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToXSplineHeading(posX, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TangentTrajectoryBuilder(trajBuilder.lineToY(posY, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToYConstantHeading(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = ConstantTrajectoryBuilder(
        trajBuilder.lineToXConstantHeading(
            posY, velConstraintOverride, accelConstraintOverride
        )
    )
    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        RestrictedTrajectoryBuilder(
            trajBuilder.lineToYLinearHeading(
                posY, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToYLinearHeading(posY, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToYSplineHeading(
                posY, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToYSplineHeading(posY, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TangentTrajectoryBuilder(trajBuilder.splineTo(pos, tangent, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = splineTo(pos, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        ConstantTrajectoryBuilder(
            trajBuilder.splineToConstantHeading(
                pos, tangent, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        splineToConstantHeading(pos, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        RestrictedTrajectoryBuilder(
            trajBuilder.splineToLinearHeading(
                pose, tangent, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = splineToLinearHeading(pose, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.splineToSplineHeading(
                pose, tangent, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = splineToSplineHeading(pose, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)

    fun build() = trajBuilder.build()
}

class TangentTrajectoryBuilder internal constructor(private val trajBuilder: TrajectoryBuilder) {
    @JvmOverloads
    fun forward(
        dist: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TangentTrajectoryBuilder(trajBuilder.forward(dist, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun forwardSplineHeading(
        dist: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.forwardSplineHeading(
                dist, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun forwardSplineHeading(
        dist: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = forwardSplineHeading(dist, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TangentTrajectoryBuilder(trajBuilder.lineToX(posX, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToXSplineHeading(
                posX, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToXSplineHeading(posX, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TangentTrajectoryBuilder(trajBuilder.lineToY(posY, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToYSplineHeading(
                posY, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToYSplineHeading(posY, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TangentTrajectoryBuilder(trajBuilder.splineTo(pos, tangent, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = splineTo(pos, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.splineToSplineHeading(
                pose, tangent, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = splineToSplineHeading(pose, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)

    fun build() = trajBuilder.build()
}

class ConstantTrajectoryBuilder internal constructor(private val trajBuilder: TrajectoryBuilder) {
    @JvmOverloads
    fun forwardConstantHeading(
        dist: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = ConstantTrajectoryBuilder(
        trajBuilder.forwardConstantHeading(
            dist, velConstraintOverride, accelConstraintOverride
        )
    )
    @JvmOverloads
    fun forwardSplineHeading(
        dist: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.forwardSplineHeading(
                dist, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun forwardSplineHeading(
        dist: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = forwardSplineHeading(dist, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToXConstantHeading(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = ConstantTrajectoryBuilder(
        trajBuilder.lineToXConstantHeading(
            posX, velConstraintOverride, accelConstraintOverride
        )
    )
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToXSplineHeading(
                posX, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToXSplineHeading(posX, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToYConstantHeading(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = ConstantTrajectoryBuilder(
        trajBuilder.lineToXConstantHeading(
            posY, velConstraintOverride, accelConstraintOverride
        )
    )
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToYSplineHeading(
                posY, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToYSplineHeading(posY, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        ConstantTrajectoryBuilder(
            trajBuilder.splineToConstantHeading(
                pos, tangent, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        splineToConstantHeading(pos, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.splineToSplineHeading(
                pose, tangent, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = splineToSplineHeading(pose, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)

    fun build() = trajBuilder.build()
}

class RestrictedTrajectoryBuilder internal constructor(private val trajBuilder: TrajectoryBuilder) {
    @JvmOverloads
    fun forwardSplineHeading(
        dist: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.forwardSplineHeading(
                dist, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun forwardSplineHeading(
        dist: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = forwardSplineHeading(dist, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToXSplineHeading(
                posX, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToXSplineHeading(posX, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToYSplineHeading(
                posY, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToYSplineHeading(posY, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.splineToSplineHeading(
                pose, tangent, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = splineToSplineHeading(pose, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)

    fun build() = trajBuilder.build()
}
