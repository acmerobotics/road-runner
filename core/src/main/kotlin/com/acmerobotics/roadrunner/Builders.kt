package com.acmerobotics.roadrunner

class PositionPathBuilder private constructor(
        val paths: List<PositionPath<ArcLength>>,
        val beginPos: Position2<DoubleNum>,
        val beginTangent: Rotation2<DoubleNum>,
) {
    constructor(
            beginPos: Position2<DoubleNum>,
            beginTangent: Rotation2<DoubleNum>,
    ) : this(listOf(), beginPos, beginTangent)

    // TODO: yikes! we really need monomorphization
    fun splineTo(pos: Position2<DoubleNum>, tangent: Rotation2<DoubleNum>): PositionPathBuilder {
//        QuinticSpline2(
//                QuinticSpline1()
//        )
        return PositionPathBuilder(paths + listOf(TODO("spline")), pos, tangent)
    }

//    fun addPath(path: PositionPath<ArcLength>): PositionPathBuilder {
//        require(path[0.0, 2])
//    }


    fun build() = CompositePositionPath(paths)
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
