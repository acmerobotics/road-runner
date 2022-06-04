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

//class PosePathBuilder(
//        val positionPath: PositionPath<ArcLength>,
//        val beginDisp: Double,
//        val prefix: List<PosePath>,
//) {
//    fun tangentTo(disp: Double): PosePathBuilder {
//        require(disp > beginDisp)
//        return PosePathBuilder(positionPath, disp, prefix +
//                listOf(TangentPath(PositionPathView(positionPath, beginDisp, disp, ))))
//    }
//
//    fun build() = CompositePosePath(prefix)
//}
