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

        val spline = ArcCurve2(QuinticSpline2(
                QuinticSpline1(
                        DualNum(doubleArrayOf(beginPos.x, beginDeriv.x, 0.0)),
                    DualNum(doubleArrayOf(pos.x, endDeriv.x, 0.0))
                ),
                QuinticSpline1(
                    DualNum(doubleArrayOf(beginPos.y, beginDeriv.y, 0.0)),
                        DualNum(doubleArrayOf(pos.y, endDeriv.y, 0.0)),
                )
        ))

        val splineEnd = spline.end(2)
        return PositionPathBuilder(
            paths + listOf(spline),
            splineEnd.value(),
            splineEnd.tangent().value(),
        )
    }

    fun build() = CompositePositionPath(paths)
}

class PosePathBuilder private constructor(
    // invariant: state encodes heading for [0.0, beginDisp)
    val posPath: PositionPath<ArcLength>,
        val beginDisp: Double,
        val state: State,
) {
    constructor(path: PositionPath<ArcLength>, beginRot: Rotation2) :
            this(path, 0.0, Lazy({ emptyList() }, beginRot))

    // bad look to do pattern matching and polymorphism at the same time :/
    sealed interface State {
        // TODO: naming
        fun rotation(): Rotation2
    }

    class Eager(val ps: List<PosePath>, val r: Rotation2Dual<ArcLength>) : State {
        override fun rotation() = r.value()
    }

    class Lazy(val f: (Rotation2Dual<ArcLength>) -> List<PosePath>, val r: Rotation2) : State {
        override fun rotation() = r
    }

//    fun checkDeriv

    // TODO: default disp to length?
    fun tangentTo(disp: Double): PosePathBuilder {
        // TODO: also require before the end?
        require(disp > beginDisp)

        val rot = posPath[disp, 4].tangent()
        val posePath = TangentPath(
            PositionPathView(posPath, beginDisp, disp - beginDisp),
            state.rotation() - rot.value(),
        )

        return PosePathBuilder(posPath, disp, Eager(
            when (state) {
                    is Eager -> {
                        // TODO: check derivatives
                        state.ps
                    }
                    is Lazy -> state.f(rot)
                }
                + listOf(posePath), rot)
        )
    }

    fun constantTo(disp: Double): PosePathBuilder {
        require(disp > beginDisp)

        val beginRot = state.rotation()
        val headingPath = ConstantHeadingPath(beginRot, disp - beginDisp)

        val posePath = HeadingPosePath(
            PositionPathView(posPath, beginDisp, disp - beginDisp),
            headingPath,
        )

        val rot = headingPath[disp, 3]

        // TODO: pull out Eager
        return PosePathBuilder(posPath, disp,
            when (state) {
                is Eager -> {
                    // TODO: check derivatives
                    Eager(state.ps + listOf(posePath), rot)
                }
                is Lazy ->
                    Eager(state.f(rot) + listOf(posePath), rot)
            })
    }

    fun lineTo(disp: Double, rot: Rotation2): PosePathBuilder {
        require(disp > beginDisp)

        val beginRot = state.rotation()
        val headingPath = LinearHeadingPath(beginRot, rot - beginRot, disp - beginDisp)

        val posePath = HeadingPosePath(
            PositionPathView(posPath, beginDisp, disp - beginDisp),
            headingPath,
        )

        val rot = headingPath[disp, 3]

        // TODO: same
        return PosePathBuilder(posPath, disp,
            when (state) {
                is Eager -> {
                    // TODO: check derivatives
                    Eager(state.ps + listOf(posePath), rot)
                }
                is Lazy ->
                    Eager(state.f(rot) + listOf(posePath), rot)
            })
    }

    fun splineTo(disp: Double, rot: Rotation2): PosePathBuilder {
        require(disp > beginDisp)

        return when (state) {
            is Eager -> PosePathBuilder(posPath, disp, Lazy({
                val headingPath = SplineHeadingPath(state.r, it, disp - beginDisp)

                val posePath = HeadingPosePath(
                    PositionPathView(posPath, beginDisp, disp - beginDisp),
                    headingPath,
                )

                state.ps + listOf(posePath)
            }, rot))

            is Lazy -> PosePathBuilder(posPath, disp, Lazy({
                val beginRot = posPath[disp, 4].tangent()

                val headingPath = SplineHeadingPath(
                    Rotation2Dual(
                        DualNum(doubleArrayOf(state.r.real, beginRot.real[1], beginRot.real[2])),
                        DualNum(doubleArrayOf(state.r.imag, beginRot.imag[1], beginRot.imag[2])),
                    ),
                    it, disp - beginDisp
                )

                val posePath = HeadingPosePath(
                    PositionPathView(posPath, beginDisp, disp - beginDisp),
                    headingPath,
                )

                state.f(beginRot) + listOf(posePath)
            }, rot))
        }
    }

    fun tangentToEnd() = tangentTo(posPath.length).build()
    fun constantToEnd() = constantTo(posPath.length).build()
    fun lineToEnd(rot: Rotation2) = lineTo(posPath.length, rot).build()
    fun splineToEnd(rot: Rotation2) = splineTo(posPath.length, rot).build()

    private fun build(): PosePath {
        require(beginDisp == posPath.length)

        return when(state) {
            is Eager -> CompositePosePath(state.ps)
            is Lazy -> {
                val beginRot = posPath[beginDisp, 4].tangent()

                CompositePosePath(state.f(
                    Rotation2Dual(
                        DualNum(doubleArrayOf(state.r.real, beginRot.real[1], beginRot.real[2])),
                        DualNum(doubleArrayOf(state.r.imag, beginRot.imag[1], beginRot.imag[2])),
                    ),
                ))
            }
        }
    }
}
