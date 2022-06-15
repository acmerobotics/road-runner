package com.acmerobotics.roadrunner

object Internal

class QuinticSpline1(
        begin: DualNum<Internal>,
        end: DualNum<Internal>,
) {
    init {
        require(begin.size >= 3)
        require(end.size >= 3)
    }

    val a = -6.0 * begin[0] - 3.0 * begin[1] - 0.5 * begin[2] +
        6.0 * end[0] - 3.0 * end[1] + 0.5 * end[2]
    val b = 15.0 * begin[0] + 8.0 * begin[1] + 1.5 * begin[2] -
        15.0 * end[0] + 7.0 * end[1] - end[2]
    val c = -10.0 * begin[0] - 6.0 * begin[1] - 1.5 * begin[2] +
        10.0 * end[0] - 4.0 * end[1] + 0.5 * end[2]
    val d = 0.5 * begin[2]
    val e = begin[1]
    val f = begin[0]

    operator fun get(t: Double, n: Int) = DualNum<Internal>(DoubleArray(n) {
        when (it) {
            0 -> ((((a * t + b) * t + c) * t + d) * t + e) * t + f
            1 -> (((5.0 * a * t + 4.0 * b) * t + 3.0 * c) * t + 2.0 * d) * t + e
            2 -> ((20.0 * a * t + 12.0 * b) * t + 6.0 * c) * t + 2.0 * d
            3 -> (60.0 * a * t + 24.0 * b) * t + 6.0 * c
            4 -> 120.0 * a * t + 24.0 * b
            5 -> 120.0 * a
            else -> 0.0
        }
    })
}

interface PositionPath<Param> {
    val length: Double
    operator fun get(param: Double, n: Int): Position2Dual<Param>

    fun begin(n: Int) = get(0.0, n)
    fun end(n: Int) = get(length, n)
}

class QuinticSpline2(
    val x: QuinticSpline1,
    val y: QuinticSpline1,
) : PositionPath<Internal> {
    override val length = 1.0
    override fun get(param: Double, n: Int) = Position2Dual(x[param, n], y[param, n])
}

class Line(
        val begin: Position2,
        val end: Position2,
) : PositionPath<ArcLength> {
    val diff = end - begin
    override val length = diff.norm()
    val dir = diff / length

    override fun get(param: Double, n: Int) =
            DualNum.variable<ArcLength>(param, n) * dir + begin
}

object ArcLength

class ArcCurve2(
    val curve: PositionPath<Internal>,
) : PositionPath<ArcLength> {
    val samples = integralScan(0.0, curve.length, 1e-6) {
        curve[it, 2].free().drop(1).norm().value()
    }
    override val length = samples.sums.last()

    fun reparam(s: Double): Double {
        val index = samples.sums.binarySearch(s)
        return if (index >= 0) {
            samples.values[index]
        } else {
            val insIndex = -(index + 1)
            when {
                insIndex <= 0 -> 0.0
                insIndex >= samples.values.size -> 1.0
                else -> {
                    val sLo = samples.sums[insIndex - 1]
                    val sHi = samples.sums[insIndex]
                    val tLo = samples.values[insIndex - 1]
                    val tHi = samples.values[insIndex]
                    lerp(s, sLo, sHi, tLo, tHi)
                }
            }
        }
    }

    override fun get(param: Double, n: Int): Position2Dual<ArcLength> {
        val t = reparam(param)
        val point = curve[t, n]

        val tValues = DoubleArray(n)
        tValues[0] = t
        if (n <= 1) return point.reparam(DualNum(tValues))

        val tDerivs = point.free().drop(1).norm().recip()
        tValues[1] = tDerivs[0]
        if (n <= 2) return point.reparam(DualNum(tValues))

        tValues[2] = tDerivs.reparam(DualNum<ArcLength>(tValues))[1]
        if (n <= 3) return point.reparam(DualNum(tValues))

        tValues[3] = tDerivs.reparam(DualNum<ArcLength>(tValues))[2]
        return point.reparam(DualNum(tValues))
    }
}

// TODO: perhaps this can be made more generic?
// could it be useful for trajectories and such? (copying is probably better tbh)
data class CompositePositionPath<Param>(
    val paths: List<PositionPath<Param>>,
    // TODO: partialSumByDouble() when?
    // actually, maybe this should be a real utility method
    val offsets: List<Double> = paths.scan(0.0) { acc, path -> acc + path.length }
) : PositionPath<Param> {
    override val length = offsets.last()

    init {
        require(paths.isNotEmpty())
    }

    override fun get(param: Double, n: Int): Position2Dual<Param> {
        if (param >= length) {
            return Position2Dual.constant(paths.last().end(1).value(), n)
        }

        for ((offset, path) in offsets.zip(paths).reversed()) {
            if (param >= offset) {
                return path[param - offset, n]
            }
        }

        return Position2Dual.constant(paths.first()[0.0, 1].value(), n)
    }
}

data class PositionPathView<Param>(
        val path: PositionPath<Param>,
        val offset: Double,
        override val length: Double,
) : PositionPath<Param> {
    override fun get(param: Double, n: Int) = path[param + offset, n]
}

// TODO: is this actually necessary with the builders?
fun <Param> splitPositionPath(path: PositionPath<Param>, cuts: List<Double>): List<PositionPath<Param>> {
    if (cuts.isEmpty()) {
        return listOf(path)
    }

    require(cuts.zip(cuts.drop(1)).all { (a, b) -> a < b })
    require(cuts.first() > 0.0)
    require(cuts.last() < path.length)

    val views = mutableListOf<PositionPath<Param>>()
    val finalBegin = cuts.fold(0.0) { begin, end ->
        views.add(PositionPathView(path, begin, end - begin))
        end
    }

    views.add(PositionPathView(path, finalBegin, path.length - finalBegin))

    return views
}

interface HeadingPath {
    val length: Double
    operator fun get(s: Double, n: Int): Rotation2Dual<ArcLength>

    fun begin(n: Int) = get(0.0, n)
    fun end(n: Int) = get(length, n)
}

class ConstantHeadingPath(
    val heading: Rotation2, override val length: Double,
) : HeadingPath {
    override fun get(s: Double, n: Int) = Rotation2Dual.constant<ArcLength>(heading, n)
}

class LinearHeadingPath(
        val begin: Rotation2,
        val angle: Double,
        override val length: Double,
) : HeadingPath {
    override fun get(s: Double, n: Int) =
            Rotation2Dual.exp(DualNum.variable<ArcLength>(s, n) / length * angle) * begin
}

class SplineHeadingPath(
        val begin: Rotation2Dual<ArcLength>,
        val end: Rotation2Dual<ArcLength>,
        override val length: Double,
) : HeadingPath {
    init {
        require(begin.size >= 3)
        require(end.size >= 3)
    }

    val spline = QuinticSpline1(
            DualNum.constant(0.0, 3),
            (end - begin).reparam(
                    // s(t) = t * len
                    DualNum.variable<Internal>(1.0, 3) * length)
    )

    override fun get(s: Double, n: Int) =
            Rotation2Dual.exp(spline[s / length, n].reparam(
                    // t(s) = s / len
                    DualNum.variable<ArcLength>(s, n) / length)) * begin
}

interface PosePath {
    val length: Double
    operator fun get(s: Double, n: Int): Transform2Dual<ArcLength>

    fun begin(n: Int) = get(0.0, n)
    fun end(n: Int) = get(length, n)
}

data class TangentPath(
        val path: PositionPath<ArcLength>,
        val offset: Double
) : PosePath {
    override val length get() = path.length

    // NOTE: n+1 guarantees enough derivatives for tangent
    override operator fun get(s: Double, n: Int) = path[s, n + 1].let {
        Transform2Dual(it.free(), it.tangent() + offset)
    }
}

data class HeadingPosePath(
        val posPath: PositionPath<ArcLength>,
        val headingPath: HeadingPath,
) : PosePath {
    override val length get() = headingPath.length

    override fun get(s: Double, n: Int) =
            Transform2Dual(posPath[s, n].free(), headingPath[s, n])
}

data class CompositePosePath(
    val paths: List<PosePath>,
    val offsets: List<Double> = paths.scan(0.0) { acc, path -> acc + path.length },
) : PosePath {
    // TODO: partialSumByDouble() when?
    override val length = offsets.last()

    init {
        require(paths.isNotEmpty())
    }

    override fun get(s: Double, n: Int): Transform2Dual<ArcLength> {
        if (s >= length) {
            return Transform2Dual.constant(paths.last().end(1).value(), n)
        }

        for ((offset, path) in offsets.zip(paths).reversed()) {
            if (s >= offset) {
                return path[s - offset, n]
            }
        }

        return Transform2Dual.constant(paths.first()[0.0, 1].value(), n)
    }
}

fun project(path: PosePath, query: Position2, init: Double): Double {
    return (1..10).fold(init) { s, _ ->
        val guess = path[s, 3].translation.bind()
        val ds = (query - guess.value()) dot guess.tangent().value().vec()
        clamp(s + ds, 0.0, path.length)
    }
}
