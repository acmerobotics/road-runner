package com.acmerobotics.roadrunner

class PositionPathBuilder private constructor(
    // invariant: beginPose and beginTangent match the end pose and tangent of paths.last()
    val paths: List<PositionPath<ArcLength>>,
    val beginPos: Position2,
    val beginTangent: Rotation2,
) {
    constructor(
        beginPos: Position2,
        beginTangent: Rotation2,
    ) : this(listOf(), beginPos, beginTangent)

    fun lineTo(pos: Position2): PositionPathBuilder {
        val line = Line(beginPos, pos)
        val lineEnd = line.end(2)
        return PositionPathBuilder(
            paths + listOf(line),
            lineEnd.value(),
            lineEnd.tangent().value(),
        )
    }

    fun splineTo(pos: Position2, tangent: Rotation2): PositionPathBuilder {
        // NOTE: First derivatives will be normalized by arc length reparam, so the
        // magnitudes need not match at knots.
        val dist = (pos - beginPos).norm()
        val beginDeriv = beginTangent.vec() * dist
        val endDeriv = tangent.vec() * dist

        val spline = ArcCurve2(
            QuinticSpline2(
                QuinticSpline1(
                    DualNum(doubleArrayOf(beginPos.x, beginDeriv.x, 0.0)),
                    DualNum(doubleArrayOf(pos.x, endDeriv.x, 0.0))
                ),
                QuinticSpline1(
                    DualNum(doubleArrayOf(beginPos.y, beginDeriv.y, 0.0)),
                    DualNum(doubleArrayOf(pos.y, endDeriv.y, 0.0)),
                )
            )
        )

        val splineEnd = spline.end(2)
        return PositionPathBuilder(
            paths + listOf(spline),
            splineEnd.value(),
            splineEnd.tangent().value(),
        )
    }

    fun build() = CompositePositionPath(paths)
}

// TODO: document a guarantee about continuity
class PosePathBuilder private constructor(
    // invariant: state encodes heading for [0.0, beginDisp)
    val posPath: PositionPath<ArcLength>,
    val beginDisp: Double,
    val state: State,
) {
    constructor(path: PositionPath<ArcLength>, beginHeading: Rotation2) :
            this(path, 0.0, Lazy({ emptyList() }, beginHeading))

    sealed interface State {
        val endHeading: Rotation2
    }

    class Eager(val paths: List<PosePath>, val endHeadingDual: Rotation2Dual<ArcLength>) : State {
        override val endHeading = endHeadingDual.value()
    }

    class Lazy(val makePaths: (Rotation2Dual<ArcLength>) -> List<PosePath>, override val endHeading: Rotation2) : State

    // TODO: keep this private?
    // pro: easier to make breaking changes in the future
    // con: more difficult to extend the builder
    private fun addEagerPosePath(disp: Double, posePath: PosePath): PosePathBuilder {
        require(disp > beginDisp)

        val beginHeadingDual = posePath.begin(3).rotation

        return PosePathBuilder(
            posPath, disp, Eager(
                when (state) {
                    is Eager -> {
                        // TODO: Rotation2.epsilonEquals?
                        require(state.endHeadingDual.real.epsilonEquals(beginHeadingDual.real))
                        require(state.endHeadingDual.imag.epsilonEquals(beginHeadingDual.imag))

                        state.paths
                    }
                    is Lazy -> state.makePaths(beginHeadingDual)
                } + listOf(posePath),
                posePath.end(3).rotation
            )
        )
    }

    private fun viewTo(disp: Double) =
        PositionPathView(posPath, beginDisp, disp - beginDisp)

    fun tangentTo(disp: Double) = addEagerPosePath(
        disp,
        TangentPath(
            viewTo(disp),
            state.endHeading - posPath[disp, 2].tangent().value()
        )
    )

    fun constantTo(disp: Double) = addEagerPosePath(
        disp,
        HeadingPosePath(
            viewTo(disp),
            ConstantHeadingPath(state.endHeading, disp - beginDisp),
        )
    )

    fun lineTo(disp: Double, heading: Rotation2) = addEagerPosePath(
        disp,
        HeadingPosePath(
            viewTo(disp),
            LinearHeadingPath(state.endHeading, heading - state.endHeading, disp - beginDisp)
        )
    )

    fun splineTo(disp: Double, heading: Rotation2): PosePathBuilder {
        require(disp > beginDisp)

        return PosePathBuilder(
            posPath, disp, Lazy(
                when (state) {
                    is Eager -> {
                        {
                            state.paths + listOf(
                                HeadingPosePath(
                                    viewTo(disp),
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

                            state.makePaths(beginHeading) + listOf(
                                HeadingPosePath(
                                    viewTo(disp),
                                    SplineHeadingPath(beginHeading, it, disp - beginDisp)
                                )
                            )
                        }
                    }
                }, heading
            )
        )
    }

    fun tangentToEnd() = tangentTo(posPath.length).build()
    fun constantToEnd() = constantTo(posPath.length).build()
    fun lineToEnd(heading: Rotation2) = lineTo(posPath.length, heading).build()
    fun splineToEnd(heading: Rotation2) = splineTo(posPath.length, heading).build()

    // NOTE: must be at the end of the pose path
    fun build(): PosePath {
        require(beginDisp == posPath.length)

        return CompositePosePath(
            when (state) {
                is Eager -> state.paths
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

class SafePosePathBuilder(val posePathBuilder: PosePathBuilder) {
    constructor(path: PositionPath<ArcLength>, beginHeading: Rotation2) :
            this(PosePathBuilder(path, beginHeading))

    fun tangentTo(disp: Double) =
        RestrictedPosePathBuilder(posePathBuilder.tangentTo(disp))
    fun constantTo(disp: Double) =
        RestrictedPosePathBuilder(posePathBuilder.constantTo(disp))
    // TODO: linearTo?
    fun lineTo(disp: Double, heading: Rotation2) =
        RestrictedPosePathBuilder(posePathBuilder.lineTo(disp, heading))

    fun splineTo(disp: Double, heading: Rotation2) =
        SafePosePathBuilder(posePathBuilder.splineTo(disp, heading))

    fun tangentToEnd() = posePathBuilder.tangentToEnd()
    fun constantToEnd() = posePathBuilder.constantToEnd()
    fun lineToEnd(heading: Rotation2) = posePathBuilder.lineToEnd(heading)
    fun splineToEnd(heading: Rotation2) = posePathBuilder.splineToEnd(heading)

    fun build() = posePathBuilder.build()
}

class RestrictedPosePathBuilder(val posePathBuilder: PosePathBuilder) {
    fun splineTo(disp: Double, heading: Rotation2) =
        SafePosePathBuilder(posePathBuilder.splineTo(disp, heading))

    fun splineToEnd(heading: Rotation2) = posePathBuilder.splineToEnd(heading)

    fun build() = posePathBuilder.build()
}
