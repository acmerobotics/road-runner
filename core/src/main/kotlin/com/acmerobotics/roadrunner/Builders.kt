package com.acmerobotics.roadrunner

import kotlin.math.PI
import kotlin.math.abs

/**
 * @usesMathJax
 *
 * Builds a sequence of \(C^2\) [CompositePositionPath]s with [Arclength] parameters.
 *
 * A new path is started whenever extending the current path would violate the continuity requirement. To manually
 * insert further path breaks, invoke this builder multiple times.
 */
class PositionPathSeqBuilder private constructor(
    private val eps: Double,
    private val paths: List<CompositePositionPath<Arclength>>,
    // invariants:
    // - segments satisfy continuity guarantees
    // - last segment ends with nextBeginPos, nextBeginTangent if it exists
    private val segments: List<PositionPath<Arclength>>,
    private val nextBeginPos: Vector2d,
    private val nextBeginTangent: Rotation2d,
) {
    constructor(
        beginPos: Vector2d,
        beginTangent: Rotation2d,
        eps: Double,
    ) : this(eps, emptyList(), emptyList(), beginPos, beginTangent)

    constructor(
        beginPos: Vector2d,
        beginTangent: Double,
        eps: Double,
    ) : this(beginPos, Rotation2d.exp(beginTangent), eps)

    fun endPath() =
        PositionPathSeqBuilder(
            eps,
            if (segments.isEmpty()) {
                paths
            } else {
                paths + listOf(CompositePositionPath(segments))
            },
            emptyList(),
            nextBeginPos,
            nextBeginTangent,
        )

    fun setTangent(newTangent: Rotation2d) =
        if (abs(nextBeginTangent - newTangent) < 1e-6) {
            this
        } else {
            val b = endPath()

            PositionPathSeqBuilder(
                b.eps,
                b.paths,
                b.segments,
                b.nextBeginPos,
                newTangent,
            )
        }
    fun setTangent(newTangent: Double) = setTangent(Rotation2d.exp(newTangent))

    private fun addSegment(seg: PositionPath<Arclength>): PositionPathSeqBuilder {
        val begin = seg.begin(2)
        val beginPos = begin.value()
        val beginTangent = begin.drop(1).value().angleCast()

        val end = seg.end(2)

        val b = if (abs(nextBeginPos.x - beginPos.x) > eps ||
            abs(nextBeginPos.y - beginPos.y) > eps ||
            abs(nextBeginTangent - beginTangent) > 1e-6
        ) {
            endPath()
        } else {
            this
        }

        return PositionPathSeqBuilder(
            b.eps,
            b.paths,
            b.segments + listOf(seg),
            end.value(),
            end.drop(1).value().angleCast(),
        )
    }

    /**
     * @usesMathJax
     *
     * Adds a line segment that goes to \(x\)-coordinate [posX].
     */
    fun lineToX(posX: Double): PositionPathSeqBuilder {
        require(abs(nextBeginTangent.real) > 1e-6) {
            "Path tangent orthogonal to the x-axis, try using lineToY() instead"
        }

        return addSegment(
            Line(
                nextBeginPos,
                Vector2d(
                    posX,
                    (posX - nextBeginPos.x) / nextBeginTangent.real * nextBeginTangent.imag + nextBeginPos.y
                )
            )
        )
    }

    /**
     * @usesMathJax
     *
     * Adds a line segment that goes to \(y\)-coordinate [posY].
     */
    fun lineToY(posY: Double): PositionPathSeqBuilder {
        require(abs(nextBeginTangent.imag) > 1e-6) {
            "Path tangent orthogonal to the y-axis, try using lineToX() instead"
        }

        return addSegment(
            Line(
                nextBeginPos,
                Vector2d(
                    (posY - nextBeginPos.y) / nextBeginTangent.imag * nextBeginTangent.real + nextBeginPos.x, posY,
                )
            )
        )
    }

    /**
     * Sets the tangent to point toward [pos], and adds a line segment in that direction.
     */
    fun strafeTo(pos: Vector2d): PositionPathSeqBuilder {
        val diff = pos - nextBeginPos
        // TODO: should we add a normalized() method or an angle() method?
        val norm = diff.norm()
        setTangent((diff / norm).angleCast())

        return addSegment(
            Line(
                nextBeginPos,
                pos
            )
        )
    }

    /**
     * Adds a spline segment to position [pos] with tangent [tangent].
     */
    fun splineTo(pos: Vector2d, tangent: Rotation2d): PositionPathSeqBuilder {
        val dist = (pos - nextBeginPos).norm()

        // NOTE: First derivatives will be normalized by arc length reparam, so the magnitudes need not match at knots.
        val beginDeriv = nextBeginTangent.vec() * dist
        val endDeriv = tangent.vec() * dist

        val spline = ArclengthReparamCurve2d(
            QuinticSpline2d(
                QuinticSpline1d(
                    DualNum(doubleArrayOf(nextBeginPos.x, beginDeriv.x, 0.0)),
                    DualNum(doubleArrayOf(pos.x, endDeriv.x, 0.0))
                ),
                QuinticSpline1d(
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

    fun build() = paths + listOf(CompositePositionPath(segments))
}

private fun <Param> DualNum<Param>.withValue(x: Double) =
    DualNum<Param>(
        DoubleArray(size()) {
            if (it == 0) {
                x
            } else {
                values()[it]
            }
        }
    )

private fun <Param> Rotation2dDual<Param>.withValue(r: Rotation2d) =
    Rotation2dDual(real.withValue(r.real), imag.withValue(r.imag))

/**
 * @usesMathJax
 *
 * Builds a sequence of [CompositePosePath]s, each guaranteeing \(C^1\) heading continuity. Requires that pose path has
 * \(C^2\) continuity.
 *
 * A new path is started whenever extending the current path would violate the continuity requirement. To manually
 * insert further path breaks, invoke this builder multiple times.
 */
class PosePathSeqBuilder private constructor(
    // precondition: posPath is C2-continuous
    private val posPath: PositionPath<Arclength>,
    // invariants:
    // - state segments satisfy continuity guarantees
    // - state encodes heading for [0.0, endDisp)
    private val posePaths: List<CompositePosePath>,
    private val endDisp: Double,
    private val state: State,
) {
    constructor(path: PositionPath<Arclength>, beginHeading: Rotation2d) :
        this(path, emptyList(), 0.0, Lazy({ emptyList() }, beginHeading))

    constructor(path: PositionPath<Arclength>, beginHeading: Double) :
        this(path, emptyList(), 0.0, Lazy({ emptyList() }, Rotation2d.exp(beginHeading)))

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

    private fun addEagerPosePath(disp: Double, segment: PosePath): PosePathSeqBuilder {
        require(endDisp <= disp && disp <= posPath.length())

        val beginHeadingDual = segment.begin(3).heading

        return when (state) {
            is Eager -> {
                fun <Param> DualNum<Param>.epsilonEquals(n: DualNum<Param>) =
                    values().zip(n.values()).all { (x, y) -> abs(x - y) < 1e-6 }

                fun <Param> Rotation2dDual<Param>.epsilonEquals(r: Rotation2dDual<Param>) =
                    real.epsilonEquals(r.real) && imag.epsilonEquals(r.imag)

                if (state.endHeadingDual.epsilonEquals(beginHeadingDual)) {
                    PosePathSeqBuilder(
                        posPath,
                        posePaths,
                        disp,
                        Eager(
                            state.segments + listOf(segment),
                            segment.end(3).heading
                        )
                    )
                } else {
                    PosePathSeqBuilder(
                        posPath,
                        posePaths + listOf(CompositePosePath(state.segments)),
                        disp,
                        Eager(
                            listOf(segment),
                            segment.end(3).heading
                        )
                    )
                }
            }

            is Lazy -> {
                PosePathSeqBuilder(
                    posPath,
                    posePaths,
                    disp,
                    Eager(
                        state.makePaths(beginHeadingDual) + listOf(segment),
                        segment.end(3).heading
                    )
                )
            }
        }
    }

    private fun viewUntil(disp: Double) =
        PositionPathView(posPath, endDisp, disp - endDisp)

    /**
     * Fills in tangent headings until displacement [disp].
     */
    fun tangentUntil(disp: Double) = addEagerPosePath(
        disp,
        TangentPath(
            viewUntil(disp),
            state.endHeading - posPath[endDisp, 2].drop(1).value().angleCast()
        )
    )

    /**
     * Fills in constant headings until displacement [disp].
     */
    fun constantUntil(disp: Double) = addEagerPosePath(
        disp,
        HeadingPosePath(
            viewUntil(disp),
            ConstantHeadingPath(state.endHeading, disp - endDisp),
        )
    )

    /**
     * Fills in headings interpolated linearly to heading [heading] at displacement [disp].
     */
    fun linearUntil(disp: Double, heading: Rotation2d) = addEagerPosePath(
        disp,
        HeadingPosePath(
            viewUntil(disp),
            LinearHeadingPath(state.endHeading, heading - state.endHeading, disp - endDisp)
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
    fun splineUntil(disp: Double, heading: Rotation2d): PosePathSeqBuilder {
        require(endDisp < disp && disp <= posPath.length())

        return PosePathSeqBuilder(
            posPath,
            posePaths,
            disp,
            Lazy(
                when (state) {
                    is Eager -> {
                        {
                            state.segments + listOf(
                                HeadingPosePath(
                                    viewUntil(disp),
                                    SplineHeadingPath(state.endHeadingDual, it, disp - endDisp),
                                )
                            )
                        }
                    }
                    is Lazy -> {
                        {
                            val beginTangent = posPath[endDisp, 4].drop(1).angleCast()
                            val beginHeading = beginTangent.withValue(state.endHeading)

                            state.makePaths(beginHeading) + listOf(
                                HeadingPosePath(
                                    viewUntil(disp),
                                    SplineHeadingPath(beginHeading, it, disp - endDisp)
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

    internal fun build(): List<CompositePosePath> {
        require(endDisp == posPath.length())

        return posePaths + listOf(
            CompositePosePath(
                when (state) {
                    is Eager -> state.segments
                    is Lazy -> {
                        val endTangent = posPath[endDisp, 4].drop(1).angleCast()
                        val endHeading = endTangent.withValue(state.endHeading)

                        state.makePaths(endHeading)
                    }
                }
            )
        )
    }
}

class PathBuilder private constructor(
    private val beginHeading: Rotation2d, // constant
    private val positionPathSeqBuilder: PositionPathSeqBuilder,
    private val headingSegments: List<PosePathSeqBuilder.(Double) -> PosePathSeqBuilder>,
    private val endHeading: Rotation2d,
) {
    constructor(beginPose: Pose2d, eps: Double) :
        this(
            beginPose.heading,
            PositionPathSeqBuilder(beginPose.position, beginPose.heading, eps),
            emptyList(),
            beginPose.heading,
        )

    private fun copy(
        positionPathSeqBuilder: PositionPathSeqBuilder,
        headingSegments: List<PosePathSeqBuilder.(Double) -> PosePathSeqBuilder>,
        endHeading: Rotation2d,
    ) =
        PathBuilder(beginHeading, positionPathSeqBuilder, headingSegments, endHeading)

    private fun copyTangent(
        positionPathSeqBuilder: PositionPathSeqBuilder,
        headingSegments: List<PosePathSeqBuilder.(Double) -> PosePathSeqBuilder>,
    ): PathBuilder {
        val lastSeg = positionPathSeqBuilder.build().last().paths.last()
        val headingDiff = lastSeg.end(2).drop(1).angleCast().value() -
            lastSeg.begin(2).drop(1).angleCast().value()
        return PathBuilder(beginHeading, positionPathSeqBuilder, headingSegments, endHeading + headingDiff)
    }

    fun setTangent(r: Rotation2d) = PathBuilder(
        beginHeading, positionPathSeqBuilder.setTangent(r),
        headingSegments, endHeading
    )
    fun setTangent(r: Double) = setTangent(Rotation2d.exp(r))

    fun setReversed(reversed: Boolean) = setTangent(
        endHeading + if (reversed) {
            PI
        } else {
            0.0
        }
    )

    fun lineToX(posX: Double) = copyTangent(
        positionPathSeqBuilder.lineToX(posX),
        headingSegments + listOf { tangentUntil(it) }
    )

    fun lineToXConstantHeading(posX: Double) =
        copy(positionPathSeqBuilder.lineToX(posX), headingSegments + listOf { constantUntil(it) }, endHeading)

    fun lineToXLinearHeading(posX: Double, heading: Rotation2d) =
        copy(positionPathSeqBuilder.lineToX(posX), headingSegments + listOf { linearUntil(it, heading) }, heading)
    fun lineToXLinearHeading(posX: Double, heading: Double) = lineToXLinearHeading(posX, Rotation2d.exp(heading))

    fun lineToXSplineHeading(posX: Double, heading: Rotation2d) =
        copy(positionPathSeqBuilder.lineToX(posX), headingSegments + listOf { splineUntil(it, heading) }, heading)
    fun lineToXSplineHeading(posX: Double, heading: Double) = lineToXSplineHeading(posX, Rotation2d.exp(heading))

    fun lineToY(posY: Double) = copyTangent(
        positionPathSeqBuilder.lineToY(posY),
        headingSegments + listOf { tangentUntil(it) }
    )

    fun lineToYConstantHeading(posY: Double) =
        copy(positionPathSeqBuilder.lineToY(posY), headingSegments + listOf { constantUntil(it) }, endHeading)

    fun lineToYLinearHeading(posY: Double, heading: Rotation2d) =
        copy(positionPathSeqBuilder.lineToY(posY), headingSegments + listOf { linearUntil(it, heading) }, heading)
    fun lineToYLinearHeading(posY: Double, heading: Double) = lineToYLinearHeading(posY, Rotation2d.exp(heading))

    fun lineToYSplineHeading(posY: Double, heading: Rotation2d) =
        copy(positionPathSeqBuilder.lineToY(posY), headingSegments + listOf { splineUntil(it, heading) }, heading)
    fun lineToYSplineHeading(posY: Double, heading: Double) = lineToYSplineHeading(posY, Rotation2d.exp(heading))

    fun strafeTo(pos: Vector2d) = copyTangent(
        positionPathSeqBuilder.strafeTo(pos),
        headingSegments + listOf { tangentUntil(it) }
    )

    fun strafeToConstantHeading(pos: Vector2d) =
        copy(positionPathSeqBuilder.strafeTo(pos), headingSegments + listOf { constantUntil(it) }, endHeading)

    fun strafeToLinearHeading(pos: Vector2d, heading: Rotation2d) =
        copy(positionPathSeqBuilder.strafeTo(pos), headingSegments + listOf { linearUntil(it, heading) }, heading)
    fun strafeToLinearHeading(pos: Vector2d, heading: Double) = strafeToLinearHeading(pos, Rotation2d.exp(heading))

    fun strafeToSplineHeading(pos: Vector2d, heading: Rotation2d) =
        copy(positionPathSeqBuilder.strafeTo(pos), headingSegments + listOf { splineUntil(it, heading) }, heading)
    fun strafeToSplineHeading(pos: Vector2d, heading: Double) = strafeToSplineHeading(pos, Rotation2d.exp(heading))

    fun splineTo(pos: Vector2d, tangent: Rotation2d) =
        copyTangent(positionPathSeqBuilder.splineTo(pos, tangent), headingSegments + listOf { tangentUntil(it) })
    fun splineTo(pos: Vector2d, tangent: Double) = splineTo(pos, Rotation2d.exp(tangent))

    fun splineToConstantHeading(pos: Vector2d, tangent: Rotation2d) =
        copy(positionPathSeqBuilder.splineTo(pos, tangent), headingSegments + listOf { constantUntil(it) }, endHeading)
    fun splineToConstantHeading(pos: Vector2d, tangent: Double) =
        splineToConstantHeading(pos, Rotation2d.exp(tangent))

    fun splineToLinearHeading(pose: Pose2d, tangent: Rotation2d) = copy(
        positionPathSeqBuilder.splineTo(pose.position, tangent),
        headingSegments + listOf { linearUntil(it, pose.heading) }, pose.heading
    )
    fun splineToLinearHeading(pose: Pose2d, tangent: Double) = splineToLinearHeading(pose, Rotation2d.exp(tangent))

    fun splineToSplineHeading(pose: Pose2d, tangent: Rotation2d) = copy(
        positionPathSeqBuilder.splineTo(pose.position, tangent),
        headingSegments + listOf { splineUntil(it, pose.heading) }, pose.heading
    )
    fun splineToSplineHeading(pose: Pose2d, tangent: Double) = splineToSplineHeading(pose, Rotation2d.exp(tangent))

    fun build(): List<CompositePosePath> {
        val posPaths = positionPathSeqBuilder.build()
        var i = 0

        val posePaths = mutableListOf<CompositePosePath>()
        var nextHeading = beginHeading
        for (posPath in posPaths) {
            var posePathSeqBuilder = PosePathSeqBuilder(posPath, nextHeading)
            for (s in posPath.offsets.drop(1)) {
                val headingSeg = headingSegments[i++]
                posePathSeqBuilder = posePathSeqBuilder.headingSeg(s)
            }

            posePaths.addAll(posePathSeqBuilder.build())

            nextHeading = posePaths.last().end(1).value().heading
        }

        return posePaths
    }
}

fun interface PoseMap {
    fun map(pose: Pose2dDual<Arclength>): Pose2dDual<Arclength>
}

fun PoseMap.map(pose: Pose2d) = map(Pose2dDual.constant(pose, 1)).value()

class IdentityPoseMap : PoseMap {
    override fun map(pose: Pose2dDual<Arclength>) = pose
}

data class MappedPosePath(
    val basePath: PosePath,
    val poseMap: PoseMap,
) : PosePath {
    override fun length() = basePath.length()
    override fun get(s: Double, n: Int) = poseMap.map(basePath[s, n])
}

data class TrajectoryBuilderParams(
    val arcLengthSamplingEps: Double,
    val profileParams: ProfileParams,
)

class TrajectoryBuilder private constructor(
    private val profileParams: ProfileParams,
    private val pathBuilder: PathBuilder,
    private val beginEndVel: Double,
    private val baseVelConstraint: VelConstraint,
    private val baseAccelConstraint: AccelConstraint,
    private val poseMap: PoseMap,
    private val velConstraints: List<VelConstraint>,
    private val accelConstraints: List<AccelConstraint>,
) {
    @JvmOverloads
    constructor(
        params: TrajectoryBuilderParams,
        beginPose: Pose2d,
        beginEndVel: Double,
        baseVelConstraint: VelConstraint,
        baseAccelConstraint: AccelConstraint,
        poseMap: PoseMap = IdentityPoseMap(),
    ) :
        this(
            params.profileParams,
            PathBuilder(beginPose, params.arcLengthSamplingEps),
            beginEndVel, baseVelConstraint, baseAccelConstraint,
            poseMap, listOf(), listOf()
        )

    private fun add(
        newPathBuilder: PathBuilder,
        velConstraintOverride: VelConstraint?,
        accelConstraintOverride: AccelConstraint?
    ) =
        TrajectoryBuilder(
            profileParams,
            newPathBuilder, beginEndVel, baseVelConstraint, baseAccelConstraint, poseMap,
            velConstraints + listOf(velConstraintOverride ?: baseVelConstraint),
            accelConstraints + listOf(accelConstraintOverride ?: baseAccelConstraint)
        )

    fun setTangent(r: Rotation2d) =
        TrajectoryBuilder(
            profileParams,
            pathBuilder.setTangent(r), beginEndVel, baseVelConstraint, baseAccelConstraint,
            poseMap, velConstraints, accelConstraints,
        )
    fun setTangent(r: Double) = setTangent(Rotation2d.exp(r))

    fun setReversed(reversed: Boolean) =
        TrajectoryBuilder(
            profileParams,
            pathBuilder.setReversed(reversed), beginEndVel, baseVelConstraint, baseAccelConstraint,
            poseMap, velConstraints, accelConstraints,
        )

    @JvmOverloads
    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToX(posX), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToXConstantHeading(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXConstantHeading(posX), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXLinearHeading(posX, heading), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXLinearHeading(posX, heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXSplineHeading(posX, heading), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXSplineHeading(posX, heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToY(posY), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToYConstantHeading(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYConstantHeading(posY), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYLinearHeading(posY, heading), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYLinearHeading(posY, heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYSplineHeading(posY, heading), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYSplineHeading(posY, heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun strafeTo(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.strafeTo(pos), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun strafeToConstantHeading(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.strafeToConstantHeading(pos), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.strafeToLinearHeading(pos, heading), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.strafeToLinearHeading(pos, heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.strafeToSplineHeading(pos, heading), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.strafeToSplineHeading(pos, heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineTo(pos, tangent), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineTo(pos, tangent), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToConstantHeading(pos, tangent), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToConstantHeading(pos, tangent), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToLinearHeading(pose, tangent), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToLinearHeading(pose, tangent), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToSplineHeading(pose, tangent), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToSplineHeading(pose, tangent), velConstraintOverride, accelConstraintOverride)

    fun build(): List<Trajectory> {
        val rawPaths = pathBuilder.build()
        val offsets = rawPaths.scan(0) { acc, rawPath -> acc + rawPath.paths.size }
        return rawPaths.zip(offsets).map { (rawPath, offset) ->
            val path = MappedPosePath(rawPath, poseMap)

            Trajectory(
                path,
                profile(
                    profileParams,
                    path, beginEndVel,
                    CompositeVelConstraint(
                        velConstraints.slice(offset until offset + rawPath.paths.size),
                        rawPath.offsets,
                    ),
                    CompositeAccelConstraint(
                        accelConstraints.slice(offset until offset + rawPath.paths.size),
                        rawPath.offsets,
                    ),
                ),
                rawPath.offsets
            )
        }
    }
}
