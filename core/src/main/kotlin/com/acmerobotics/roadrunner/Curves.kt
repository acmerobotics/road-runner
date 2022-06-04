package com.acmerobotics.roadrunner

// TODO: is this okay being an object on the Java side?
object Internal

// TODO: should this go somewhere else?
private fun lerp(x: Double, fromLo: Double, fromHi: Double, toLo: Double, toHi: Double) =
    toLo + (x - fromLo) * (toHi - toLo) / (fromHi - fromLo)

class QuinticSpline1(
        begin: DualNum<Internal>,
        end: DualNum<Internal>,
) {
    init {
        require(begin.size >= 3)
        require(end.size >= 3)
    }

    // TODO: this needs a test
    val a = -6.0 * begin[0] - 3.0 * begin[1] - 0.5 * begin[2] +
        6.0 * end[0] - 3.0 * end[1] + 0.5 * end[2]
    val b = 15.0 * begin[0] + 8.0 * begin[1] + 1.5 * begin[2] -
        15.0 * end[0] + 7.0 * end[1] - end[2]
    val c = -10.0 * begin[0] - 6.0 * begin[1] - 1.5 * begin[2] +
        10.0 * end[0] - 4.0 * end[1] + 0.5 * end[2]
    val d = 0.5 * begin[2]
    val e = begin[1]
    val f = begin[0]

    // TODO: this needs a test
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

// TODO: computing unnecessary derivatives is unnecessarily expensive
// this is where laziness would be very helpful
interface PositionPath<Param> {
    // TODO: can I stomach length here?
    val length: Double
    operator fun get(param: Double, n: Int): Position2Dual<Param>
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

data class ScanResult(
    val values: List<Double>,
    val sums: List<Double>,
)

// implementation of adaptsim from Gander and Gautschi
// TODO: test this
fun integralScan(f: (Double) -> Double, a: Double, b: Double, eps: Double): ScanResult {
    val m = (a + b) / 2
    val fa = f(a); val fm = f(m); val fb = f(b)

    // TODO: the autoformatter hates me
    var i = (b - a) / 8 * (
        fa + fm + fb +
            f(a + 0.9501 * (b - a)) +
            f(a + 0.2311 * (b - a)) +
            f(a + 0.6068 * (b - a)) +
            f(a + 0.4860 * (b - a)) +
            f(a + 0.8913 * (b - a))
        )
    if (i == 0.0) {
        i = b - a
    }
    i *= eps / Math.ulp(1.0)

    val values = mutableListOf(0.0)
    val sums = mutableListOf(0.0)

    fun helper(a: Double, m: Double, b: Double, fa: Double, fm: Double, fb: Double) {
        val h = (b - a) / 4
        val ml = a + h; val mr = b - h
        val fml = f(ml); val fmr = f(mr)
        var i1 = h / 1.5 * (fa + 4 * fm + fb)
        val i2 = h / 3 * (fa + 4 * (fml + fmr) + 2 * fm + fb)
        i1 = (16 * i2 - i1) / 15
        if (i + (i1 - i2) == i || m <= a || b <= m) {
            values.add(b)
            sums.add(sums.last() + i1)
        } else {
            helper(a, ml, m, fa, fml, fm)
            helper(m, mr, b, fm, fmr, fb)
        }
    }

    helper(a, m, b, fa, fm, fb)

    return ScanResult(values, sums)
}

object ArcLength

class ArcCurve2(
    val curve: PositionPath<Internal>,
) : PositionPath<ArcLength> {
    val samples = integralScan({
        // TODO: there should be a method to extract the "value" of a dual num
        curve[it, 2].free().drop(1).norm().values[0]
    }, 0.0, curve.length, 1e-6)
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

        val tDerivs = point.free().drop(1).norm().recip()

        // TODO: I think we just accept the ouroboros
        // is there any unnecessary computation?
        val dtds = tDerivs.values[0]
        val d2tds2 = tDerivs.reparam(DualNum<ArcLength>(doubleArrayOf(t, dtds))).values[1]
        val d3tds3 = tDerivs.reparam(DualNum<ArcLength>(doubleArrayOf(t, dtds, d2tds2))).values[2]

        return point.reparam(DualNum(doubleArrayOf(t, dtds, d2tds2, d3tds3)))
    }
}

// TODO: perhaps this can be made more generic?
// could it be useful for trajectories and such? (copying is probably better tbh)
class CompositePositionPath<Param>(val paths: List<PositionPath<Param>>) : PositionPath<Param> {
    // TODO: partialSumByDouble() when?
    val offsets = paths.scan(0.0) { acc, path -> acc + path.length }
    override val length = offsets.last()

    init {
        require(paths.isNotEmpty())
    }

    override fun get(param: Double, n: Int): Position2Dual<Param> {
        if (param < 0.0) {
            // TODO: asConst() would help (or just const() perhaps)
            val s = paths.first()[0.0, 1]
            return Position2Dual.constant(s.x.values[0], s.y.values[0], n)
        }

        for ((offset, path) in offsets.zip(paths)) {
            if (param < offset) {
                return path[param - offset, n]
            }
        }

        // TODO: see TODO above
        val s = paths.last()[paths.last().length, 1]
        return Position2Dual.constant(s.x.values[0], s.y.values[0], n)
    }
}

class PositionPathView<Param>(
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
    operator fun get(s: Double, n: Int): Rotation2Dual<ArcLength>
}

class ConstantHeadingPath(
        val heading: Rotation2,
) : HeadingPath {
    override fun get(s: Double, n: Int) = heading.constant<ArcLength>(n)
}

class LinearHeadingPath(
        val begin: Rotation2,
        val angle: Double,
        val length: Double,
) : HeadingPath {
    override fun get(s: Double, n: Int) =
            Rotation2Dual.exp(DualNum.variable<ArcLength>(s, n) / length * angle) * begin
}

class SplineHeadingPath(
        val begin: Rotation2Dual<ArcLength>,
        val end: Rotation2Dual<ArcLength>,
        val length: Double,
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
}

class TangentPath(
        val path: PositionPath<ArcLength>,
        val offset: Rotation2
) : PosePath {
    override val length get() = path.length

    // TODO: the n+1 is an annoying leak but probably an acceptable price for eagerness
    override operator fun get(s: Double, n: Int) = path[s, n + 1].let {
        Transform2Dual(it.free(), it.tangent() * offset)
    }
}

class HeadingPosePath(
        val posPath: PositionPath<ArcLength>,
        val headingPath: HeadingPath,
) : PosePath {
    override val length get() = posPath.length

    override fun get(s: Double, n: Int) =
            Transform2Dual(posPath[s, n].free(), headingPath[s, n])
}

class CompositePosePath(val paths: List<PosePath>) : PosePath {
    // TODO: partialSumByDouble() when?
    val offsets = paths.scan(0.0) { acc, path -> acc + path.length }
    override val length = offsets.last()

    init {
        require(paths.isNotEmpty())
    }

    override fun get(s: Double, n: Int): Transform2Dual<ArcLength> {
        if (s < 0.0) {
            // TODO: asConst() would help (or just const() perhaps)
//            val s = paths.first()[0.0, 1]
//            return Transform2.constant(s.x.values[0], s.y.values[0], n)
            TODO("really need proper constant support")
//            paths.first()[0.0, 1].
        }

        for ((offset, path) in offsets.zip(paths)) {
            if (s < offset) {
                return path[s - offset, n]
            }
        }

        // TODO: see TODO above
//        val s = paths.last()[paths.last().maxParam, 1]
//        return Position2.constant(s.x.values[0], s.y.values[0], n)
        TODO("proper constant support")
    }
}
