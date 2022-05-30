package com.acmerobotics.roadrunner

data class Position2<N : Num<N>>(val x: N, val y: N) {
//    companion object {
//        fun <Origin, Point, Param> constant(x: Double, y: Double, n: Int) = Position2<Origin, Point, DualNum<Param>>(
//            DualNum.constant(x, n), DualNum.constant(y, n))
//
//        fun <Origin> origin() = Position2<Origin, Origin, DoubleNum>(DoubleNum(0.0), DoubleNum(0.0))
//        fun <Origin, Param> origin(n : Int) = Position2<Origin, Origin, DualNum<Param>>(DualNum.constant(0.0, n), DualNum.constant(0.0, n))
//
//        fun <Origin, Point, N : Num<N>> bind(v: Vector2<N>) = Position2<Origin, Point, N>(v.x, v.y)
//    }
//
//    operator fun <End> minus(other: Position2<Origin, End, N>) = Vector2(x - other.x, y - other.y)
//    operator fun <End> plus(diff: Vector2<N>) = Position2<Origin, End, N>(x + diff.x, y + diff.y)
//
//    infix fun <OtherPoint> distTo(other: Position2<Origin, OtherPoint, N>) = (this - other).norm()
//
//    fun free() = Vector2(x, y)
}

