package com.acmerobotics.roadrunner

class PositionPathBuilder private constructor(
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
        return PositionPathBuilder(paths + listOf(line), pos,
                line[0.0, 2].tangent().constant())
    }

    fun splineTo(pos: Position2, tangent: Rotation2): PositionPathBuilder {
        val dist = (pos - beginPos).norm()
        val beginDeriv = beginTangent.vec() * dist
        val endDeriv = tangent.vec() * dist
        val spline = ArcCurve2(QuinticSpline2(
                QuinticSpline1(
                        DualNum(doubleArrayOf(beginPos.x, beginDeriv.x, 0.0)),
                        DualNum(doubleArrayOf(beginPos.y, beginDeriv.y, 0.0)),
                ),
                QuinticSpline1(
                        DualNum(doubleArrayOf(pos.x, endDeriv.x, 0.0)),
                        DualNum(doubleArrayOf(pos.y, endDeriv.y, 0.0))
                )
        ))
        return PositionPathBuilder(paths + listOf(spline), pos, tangent)
    }

    fun build() = CompositePositionPath(paths)
}

sealed interface BuilderState
class Fixed(val ps: List<PosePath>, val r: Rotation2Dual<ArcLength>) : BuilderState
class Flexible(val f: (Rotation2Dual<ArcLength>) -> List<PosePath>) : BuilderState

class PosePathBuilder(
    val posPath: PositionPath<ArcLength>,
//        val beginRot: Rotation2Dual<ArcLength>,
        val beginDisp: Double,
        val state: BuilderState,
) {
    fun tangentTo(disp: Double): PosePathBuilder {
        require(disp > beginDisp)

        when (state) {
            is Fixed -> PosePathBuilder()
        }

        return PosePathBuilder(
            posPath,
//            posPath[disp, 3].tangent(),
            disp,
        ) {
//            segmentsFun(beginRot)

        }
//        return PosePathBuilder(positionPath, disp, prefix +
//                listOf(TangentPath(PositionPathView(positionPath, beginDisp, disp - beginDisp),
//                Rotation2.exp(0.0))))
    }

    fun line(disp: Double, angle: Double): PosePathBuilder {
        require(disp > beginDisp)
        return PosePathBuilder(posPath, disp) {
            LinearHeadingPath()
        }
    }

//    fun lineTo(disp: Double, endRot: Rotation2) =
//        line(disp, endRot - )

    fun build() = CompositePosePath(posePathCont())
}
