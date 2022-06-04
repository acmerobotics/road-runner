package com.acmerobotics.roadrunner

class PositionPathBuilder(
        val beginPos: Position2<DoubleNum>,
        val beginTangent: Rotation2<DoubleNum>,
        val prefix: List<PositionPath<ArcLength>> = listOf(),
) {
    // TODO: yikes! we really need monomorphization
    fun splineTo(pos: Position2<DoubleNum>, tangent: Rotation2<DoubleNum>): PositionPathBuilder {
//        QuinticSpline2(
//                QuinticSpline1()
//        )
        return PositionPathBuilder(pos, tangent, prefix + listOf(TODO("spline")))
    }

    fun build() = CompositePositionPath(prefix)
}

class PosePathBuilder(
        val positionPath: PositionPath<ArcLength>,
        val beginDisp: Double,
        val prefix: List<PosePath>,
) {
    fun tangentTo(disp: Double): PosePathBuilder {
        require(disp > beginDisp)
        return PosePathBuilder(positionPath, disp, prefix +
                listOf(TangentPath(PositionPathView(positionPath, beginDisp, disp, ))))
    }

    fun build() = CompositePosePath(prefix)
}
