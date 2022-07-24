package com.acmerobotics.roadrunner

import kotlinx.collections.immutable.PersistentList
import kotlinx.collections.immutable.persistentListOf
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
    private val segments: PersistentList<PositionPath<Arclength>>,
    private val nextBeginPos: Position2,
    private val nextBeginTangent: Rotation2,
    private val eps: Double,
) {

    /**
     * @usesMathJax
     *
     * Exception thrown when a path segment is added that doesn't maintain \(C^1\) position continuity.
     */
    class PathContinuityException : RuntimeException()

    constructor(
        beginPos: Position2,
        beginTangent: Rotation2,
        eps: Double,
    ) : this(persistentListOf(), beginPos, beginTangent, eps)

    constructor(
        beginPos: Position2,
        beginTangent: Double,
        eps: Double,
    ) : this(persistentListOf(), beginPos, Rotation2.exp(beginTangent), eps)

    private fun addLine(line: Line): PositionPathBuilder {
        val lineEnd = line.end(2)
        return PositionPathBuilder(
            segments.add(line),
            lineEnd.value(),
            lineEnd.tangent().value(),
            eps
        )
    }

    /**
     * Adds a line segment that goes forward distance [dist].
     */
    fun forward(dist: Double) =
        addLine(Line(nextBeginPos, nextBeginPos + nextBeginTangent.vec() * dist))

    /**
     * @usesMathJax
     *
     * Adds a line segment that goes forward to \(x\)-coordinate [posX].
     */
    fun lineToX(posX: Double) =
        addLine(
            Line(
                nextBeginPos,
                Position2(
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
    fun lineToY(posY: Double) = addLine(
        Line(
            nextBeginPos,
            Position2(
                (posY - nextBeginPos.y) / nextBeginTangent.imag * nextBeginTangent.real + nextBeginPos.x, posY,
            )
        )
    )

    /**
     * Adds a spline segment to position [pos] with tangent [tangent].
     */
    fun splineTo(pos: Position2, tangent: Rotation2): PositionPathBuilder {
        // note: First derivatives will be normalized by arc length reparam, so the magnitudes need not match at knots.
        val dist = (pos - nextBeginPos).norm()
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

        val splineEnd = spline.end(2)
        return PositionPathBuilder(
            segments.add(spline),
            splineEnd.value(),
            splineEnd.tangent().value(),
            eps,
        )
    }

    /**
     * Adds a spline segment to position [pos] with tangent [tangent].
     */
    fun splineTo(pos: Position2, tangent: Double) = splineTo(pos, Rotation2.exp(tangent))

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
    constructor(path: PositionPath<Arclength>, beginHeading: Rotation2) :
        this(path, 0.0, Lazy({ persistentListOf() }, beginHeading))

    constructor(path: PositionPath<Arclength>, beginHeading: Double) :
        this(path, 0.0, Lazy({ persistentListOf() }, Rotation2.exp(beginHeading)))

    private sealed interface State {
        val endHeading: Rotation2
    }

    private class Eager(
        val segments: PersistentList<PosePath>,
        val endHeadingDual: Rotation2Dual<Arclength>
    ) : State {
        override val endHeading = endHeadingDual.value()
    }

    private class Lazy(
        val makePaths: (Rotation2Dual<Arclength>) -> PersistentList<PosePath>,
        override val endHeading: Rotation2
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

                        fun <Param> Rotation2Dual<Param>.epsilonEquals(r: Rotation2Dual<Param>) =
                            real.epsilonEquals(r.real) && imag.epsilonEquals(r.imag)

                        if (!state.endHeadingDual.epsilonEquals(beginHeadingDual)) {
                            throw RotationContinuityException()
                        }

                        state.segments
                    }
                    is Lazy -> state.makePaths(beginHeadingDual)
                }.add(segment),
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
            state.endHeading - posPath[disp, 2].tangent().value()
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
    fun linearUntil(disp: Double, heading: Rotation2) = addEagerPosePath(
        disp,
        HeadingPosePath(
            viewUntil(disp),
            LinearHeadingPath(state.endHeading, heading - state.endHeading, disp - beginDisp)
        )
    )

    /**
     * Fills in headings interpolated linearly to heading [heading] at displacement [disp].
     */
    fun linearUntil(disp: Double, heading: Double) = linearUntil(disp, Rotation2.exp(heading))

    /**
     * @usesMathJax
     *
     * Fills in headings interpolated with a spline to heading [heading] at displacement [disp].
     *
     * Flexibility in the choice of spline endpoint derivatives allows [splineUntil] to both precede and succeed any
     * other heading segment. And in fact the heading at both knots will be \(C^2\)-continuous.
     */
    fun splineUntil(disp: Double, heading: Rotation2): PosePathBuilder {
        require(disp > beginDisp)

        return PosePathBuilder(
            posPath, disp,
            Lazy(
                when (state) {
                    is Eager -> {
                        {
                            state.segments.add(
                                HeadingPosePath(
                                    viewUntil(disp),
                                    SplineHeadingPath(state.endHeadingDual, it, disp - beginDisp),
                                )
                            )
                        }
                    }
                    is Lazy -> {
                        {
                            val beginTangent = posPath[beginDisp, 4].tangent()
                            val beginHeading = Rotation2Dual.exp(
                                beginTangent.log().drop(1)
                                    .addFirst(state.endHeading.log())
                            )

                            state.makePaths(beginHeading).add(
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
    fun splineUntil(disp: Double, heading: Double) = splineUntil(disp, Rotation2.exp(heading))

    fun tangentUntilEnd() = tangentUntil(posPath.length()).build()
    fun constantUntilEnd() = constantUntil(posPath.length()).build()
    fun linearUntilEnd(heading: Rotation2) = linearUntil(posPath.length(), heading).build()
    fun linearUntilEnd(heading: Double) = linearUntilEnd(Rotation2.exp(heading))
    fun splineUntilEnd(heading: Rotation2) = splineUntil(posPath.length(), heading).build()
    fun splineUntilEnd(heading: Double) = splineUntilEnd(Rotation2.exp(heading))

    internal fun build(): CompositePosePath {
        require(beginDisp == posPath.length())

        return CompositePosePath(
            when (state) {
                is Eager -> state.segments
                is Lazy -> {
                    val endTangent = posPath[beginDisp, 4].tangent()
                    val endHeading = Rotation2Dual.exp(
                        endTangent.log().drop(1)
                            .addFirst(state.endHeading.log())
                    )

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
    constructor(path: PositionPath<Arclength>, beginHeading: Rotation2) :
        this(PosePathBuilder(path, beginHeading))
    constructor(path: PositionPath<Arclength>, beginHeading: Double) :
        this(PosePathBuilder(path, beginHeading))

    fun tangentUntil(disp: Double) =
        RestrictedPosePathBuilder(posePathBuilder.tangentUntil(disp))
    fun constantUntil(disp: Double) =
        RestrictedPosePathBuilder(posePathBuilder.constantUntil(disp))
    fun linearUntil(disp: Double, heading: Rotation2) =
        RestrictedPosePathBuilder(posePathBuilder.linearUntil(disp, heading))
    fun linearUntil(disp: Double, heading: Double) =
        RestrictedPosePathBuilder(posePathBuilder.linearUntil(disp, heading))

    fun splineUntil(disp: Double, heading: Rotation2) =
        SafePosePathBuilder(posePathBuilder.splineUntil(disp, heading))
    fun splineUntil(disp: Double, heading: Double) =
        SafePosePathBuilder(posePathBuilder.splineUntil(disp, heading))

    fun tangentUntilEnd() = posePathBuilder.tangentUntilEnd()
    fun constantUntilEnd() = posePathBuilder.constantUntilEnd()
    fun linearUntilEnd(heading: Rotation2) = posePathBuilder.linearUntilEnd(heading)
    fun linearUntilEnd(heading: Double) = posePathBuilder.linearUntilEnd(heading)
    fun splineUntilEnd(heading: Rotation2) = posePathBuilder.splineUntilEnd(heading)
    fun splineUntilEnd(heading: Double) = posePathBuilder.splineUntilEnd(heading)
}

/**
 * Do not instantiate directly. See [SafePosePathBuilder].
 */
class RestrictedPosePathBuilder internal constructor(private val posePathBuilder: PosePathBuilder) {
    fun splineUntil(disp: Double, heading: Rotation2) =
        SafePosePathBuilder(posePathBuilder.splineUntil(disp, heading))
    fun splineUntil(disp: Double, heading: Double) =
        SafePosePathBuilder(posePathBuilder.splineUntil(disp, heading))

    fun splineUntilEnd(heading: Rotation2) = posePathBuilder.splineUntilEnd(heading)
    fun splineUntilEnd(heading: Double) = posePathBuilder.splineUntilEnd(heading)
}

class PathBuilder private constructor(
    private val posPathBuilder: PositionPathBuilder,
    private val posePathBuilder: (CompositePositionPath<Arclength>, Int) -> PosePathBuilder
) {
    constructor(beginPose: Transform2, beginTangent: Rotation2, eps: Double) :
        this(
            PositionPathBuilder(beginPose.trans.bind(), beginTangent, eps),
            { path, _ -> PosePathBuilder(path, beginPose.rot) }
        )
    constructor(beginPose: Transform2, beginTangent: Double, eps: Double) :
        this(beginPose, Rotation2.exp(beginTangent), eps)

    private fun addHeadingSegment(f: PosePathBuilder.(Double) -> PosePathBuilder): (
        CompositePositionPath<Arclength>,
        Int
    ) -> PosePathBuilder = { path, i ->
        posePathBuilder(path, i - 1).f(path.offsets[i])
    }

    fun forward(dist: Double) = PathBuilder(posPathBuilder.forward(dist), addHeadingSegment { this.tangentUntil(it) })

    fun forwardConstantHeading(dist: Double) =
        PathBuilder(posPathBuilder.forward(dist), addHeadingSegment { this.constantUntil(it) })

    fun forwardLinearHeading(dist: Double, heading: Rotation2) =
        PathBuilder(posPathBuilder.forward(dist), addHeadingSegment { this.linearUntil(it, heading) })
    fun forwardLinearHeading(dist: Double, heading: Double) = forwardLinearHeading(dist, Rotation2.exp(heading))

    fun forwardSplineHeading(dist: Double, heading: Rotation2) =
        PathBuilder(posPathBuilder.forward(dist), addHeadingSegment { this.splineUntil(it, heading) })
    fun forwardSplineHeading(dist: Double, heading: Double) = forwardSplineHeading(dist, Rotation2.exp(heading))

    fun lineToX(posX: Double) = PathBuilder(posPathBuilder.lineToX(posX), addHeadingSegment { this.tangentUntil(it) })

    fun lineToXConstantHeading(posX: Double) =
        PathBuilder(posPathBuilder.lineToX(posX), addHeadingSegment { this.constantUntil(it) })

    fun lineToXLinearHeading(posX: Double, heading: Rotation2) =
        PathBuilder(posPathBuilder.lineToX(posX), addHeadingSegment { this.linearUntil(it, heading) })
    fun linetoXLinearHeading(posX: Double, heading: Double) = lineToXLinearHeading(posX, Rotation2.exp(heading))

    fun lineToXSplineHeading(posX: Double, heading: Rotation2) =
        PathBuilder(posPathBuilder.lineToX(posX), addHeadingSegment { this.splineUntil(it, heading) })
    fun lineToXSplineHeading(posX: Double, heading: Double) = lineToXSplineHeading(posX, Rotation2.exp(heading))

    fun lineToY(posY: Double) = PathBuilder(posPathBuilder.lineToY(posY), addHeadingSegment { this.tangentUntil(it) })

    fun lineToYConstantHeading(posY: Double) =
        PathBuilder(posPathBuilder.lineToY(posY), addHeadingSegment { this.constantUntil(it) })

    fun lineToYLinearHeading(posY: Double, heading: Rotation2) =
        PathBuilder(posPathBuilder.lineToY(posY), addHeadingSegment { this.linearUntil(it, heading) })
    fun linetoYLinearHeading(posY: Double, heading: Double) = lineToYLinearHeading(posY, Rotation2.exp(heading))

    fun lineToYSplineHeading(posY: Double, heading: Rotation2) =
        PathBuilder(posPathBuilder.lineToY(posY), addHeadingSegment { this.splineUntil(it, heading) })
    fun lineToYSplineHeading(posY: Double, heading: Double) = lineToYSplineHeading(posY, Rotation2.exp(heading))

    fun splineTo(pos: Position2, tangent: Rotation2) =
        PathBuilder(posPathBuilder.splineTo(pos, tangent), addHeadingSegment { this.tangentUntil(it) })
    fun splineTo(pos: Position2, tangent: Double) = splineTo(pos, Rotation2.exp(tangent))

    fun splineToConstantHeading(pos: Position2, tangent: Rotation2) =
        PathBuilder(posPathBuilder.splineTo(pos, tangent), addHeadingSegment { this.constantUntil(it) })
    fun splineToConstantHeading(pos: Position2, tangent: Double) = splineToConstantHeading(pos, Rotation2.exp(tangent))

    fun splineToLinearHeading(pose: Transform2, tangent: Rotation2) = PathBuilder(
        posPathBuilder.splineTo(pose.trans.bind(), tangent), addHeadingSegment { this.linearUntil(it, pose.rot) }
    )
    fun splineToLinearHeading(pose: Transform2, tangent: Double) = splineToLinearHeading(pose, Rotation2.exp(tangent))

    fun splineToSplineHeading(pose: Transform2, tangent: Rotation2) = PathBuilder(
        posPathBuilder.splineTo(pose.trans.bind(), tangent), addHeadingSegment { this.splineUntil(it, pose.rot) }
    )
    fun splineToSplineHeading(pose: Transform2, tangent: Double) = splineToSplineHeading(pose, Rotation2.exp(tangent))

    fun build(): CompositePosePath {
        val posPath = posPathBuilder.build()
        return posePathBuilder(posPath, posPath.offsets.lastIndex).build()
    }
}

class SafePathBuilder internal constructor(private val pathBuilder: PathBuilder) {
    constructor(beginPose: Transform2, beginTangent: Rotation2, eps: Double) :
        this(PathBuilder(beginPose, beginTangent, eps))
    constructor(beginPose: Transform2, beginTangent: Double, eps: Double) :
        this(beginPose, Rotation2.exp(beginTangent), eps)

    fun forward(dist: Double) = RestrictedPathBuilder(pathBuilder.forward(dist))
    fun forwardConstantHeading(dist: Double) = RestrictedPathBuilder(pathBuilder.forwardConstantHeading(dist))
    fun forwardLinearHeading(dist: Double, heading: Rotation2) =
        RestrictedPathBuilder(pathBuilder.forwardLinearHeading(dist, heading))
    fun forwardLinearHeading(dist: Double, heading: Double) = forwardLinearHeading(dist, Rotation2.exp(heading))
    fun forwardSplineHeading(dist: Double, heading: Rotation2) =
        SafePathBuilder(pathBuilder.forwardSplineHeading(dist, heading))
    fun forwardSplineHeading(dist: Double, heading: Double) = forwardSplineHeading(dist, Rotation2.exp(heading))

    fun lineToX(posX: Double) = RestrictedPathBuilder(pathBuilder.lineToX(posX))
    fun lineToXConstantHeading(posX: Double) = RestrictedPathBuilder(pathBuilder.lineToXConstantHeading(posX))
    fun lineToXLinearHeading(posX: Double, heading: Rotation2) =
        RestrictedPathBuilder(pathBuilder.lineToXLinearHeading(posX, heading))
    fun linetoXLinearHeading(posX: Double, heading: Double) = lineToXLinearHeading(posX, Rotation2.exp(heading))
    fun lineToXSplineHeading(posX: Double, heading: Rotation2) =
        SafePathBuilder(pathBuilder.lineToXSplineHeading(posX, heading))
    fun lineToXSplineHeading(posX: Double, heading: Double) = lineToXSplineHeading(posX, Rotation2.exp(heading))

    fun lineToY(posY: Double) = RestrictedPathBuilder(pathBuilder.lineToY(posY))
    fun lineToYConstantHeading(posY: Double) = RestrictedPathBuilder(pathBuilder.lineToXConstantHeading(posY))
    fun lineToYLinearHeading(posY: Double, heading: Rotation2) =
        RestrictedPathBuilder(pathBuilder.lineToYLinearHeading(posY, heading))
    fun linetoYLinearHeading(posY: Double, heading: Double) = lineToYLinearHeading(posY, Rotation2.exp(heading))
    fun lineToYSplineHeading(posY: Double, heading: Rotation2) =
        SafePathBuilder(pathBuilder.lineToYSplineHeading(posY, heading))
    fun lineToYSplineHeading(posY: Double, heading: Double) = lineToYSplineHeading(posY, Rotation2.exp(heading))

    fun splineTo(pos: Position2, tangent: Rotation2) = RestrictedPathBuilder(pathBuilder.splineTo(pos, tangent))
    fun splineTo(pos: Position2, tangent: Double) = splineTo(pos, Rotation2.exp(tangent))
    fun splineToConstantHeading(pos: Position2, tangent: Rotation2) =
        RestrictedPathBuilder(pathBuilder.splineToConstantHeading(pos, tangent))
    fun splineToConstantHeading(pos: Position2, tangent: Double) = splineToConstantHeading(pos, Rotation2.exp(tangent))
    fun splineToLinearHeading(pose: Transform2, tangent: Rotation2) =
        RestrictedPathBuilder(pathBuilder.splineToLinearHeading(pose, tangent))
    fun splineToLinearHeading(pose: Transform2, tangent: Double) = splineToLinearHeading(pose, Rotation2.exp(tangent))
    fun splineToSplineHeading(pose: Transform2, tangent: Rotation2) =
        SafePathBuilder(pathBuilder.splineToSplineHeading(pose, tangent))
    fun splineToSplineHeading(pose: Transform2, tangent: Double) = splineToSplineHeading(pose, Rotation2.exp(tangent))

    fun build() = pathBuilder.build()
}

class RestrictedPathBuilder internal constructor(private val pathBuilder: PathBuilder) {
    fun forwardSplineHeading(dist: Double, heading: Rotation2) =
        SafePathBuilder(pathBuilder.forwardSplineHeading(dist, heading))
    fun forwardSplineHeading(dist: Double, heading: Double) = forwardSplineHeading(dist, Rotation2.exp(heading))

    fun lineToXSplineHeading(posX: Double, heading: Rotation2) =
        SafePathBuilder(pathBuilder.lineToXSplineHeading(posX, heading))
    fun lineToXSplineHeading(posX: Double, heading: Double) = lineToXSplineHeading(posX, Rotation2.exp(heading))

    fun lineToYSplineHeading(posY: Double, heading: Rotation2) =
        SafePathBuilder(pathBuilder.lineToYSplineHeading(posY, heading))
    fun lineToYSplineHeading(posY: Double, heading: Double) = lineToYSplineHeading(posY, Rotation2.exp(heading))

    fun splineToSplineHeading(pose: Transform2, tangent: Rotation2) =
        SafePathBuilder(pathBuilder.splineToSplineHeading(pose, tangent))
    fun splineToSplineHeading(pose: Transform2, tangent: Double) = splineToSplineHeading(pose, Rotation2.exp(tangent))

    fun build() = pathBuilder.build()
}
