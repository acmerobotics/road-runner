@file:JvmName("Curves")

package com.acmerobotics.roadrunner

import kotlinx.collections.immutable.PersistentList
import kotlinx.collections.immutable.toPersistentList

class Internal

data class QuinticSpline1(
    @JvmField
    val a: Double,
    @JvmField
    val b: Double,
    @JvmField
    val c: Double,
    @JvmField
    val d: Double,
    @JvmField
    val e: Double,
    @JvmField
    val f: Double,
) {
    constructor(
        begin: DualNum<Internal>,
        end: DualNum<Internal>,
    ) : this(
        // TODO: surely there's a better way to do this
        require(begin.size >= 3).let {
            require(end.size >= 3).let {
                -6.0 * begin[0] - 3.0 * begin[1] - 0.5 * begin[2] +
                        6.0 * end[0] - 3.0 * end[1] + 0.5 * end[2]
            }
        },
        15.0 * begin[0] + 8.0 * begin[1] + 1.5 * begin[2] -
                15.0 * end[0] + 7.0 * end[1] - end[2],
        -10.0 * begin[0] - 6.0 * begin[1] - 1.5 * begin[2] +
                10.0 * end[0] - 4.0 * end[1] + 0.5 * end[2],
        0.5 * begin[2],
        begin[1],
        begin[0],
    )

    operator fun get(t: Double, n: Int) = DualNum<Internal>(
        DoubleArray(n) {
            when (it) {
                0 -> ((((a * t + b) * t + c) * t + d) * t + e) * t + f
                1 -> (((5.0 * a * t + 4.0 * b) * t + 3.0 * c) * t + 2.0 * d) * t + e
                2 -> ((20.0 * a * t + 12.0 * b) * t + 6.0 * c) * t + 2.0 * d
                3 -> (60.0 * a * t + 24.0 * b) * t + 6.0 * c
                4 -> 120.0 * a * t + 24.0 * b
                5 -> 120.0 * a
                else -> 0.0
            }
        }
    )
}

interface PositionPath<Param> {
    // TODO: consider making this an accessor like length()
    // then length can be a proper JVM field in the implementing classes
    val length: Double
    operator fun get(param: Double, n: Int): Position2Dual<Param>

    fun begin(n: Int) = get(0.0, n)
    fun end(n: Int) = get(length, n)
}

data class QuinticSpline2(
    @JvmField
    val x: QuinticSpline1,
    @JvmField
    val y: QuinticSpline1,
) : PositionPath<Internal> {
    override val length = 1.0
    override fun get(param: Double, n: Int) = Position2Dual(x[param, n], y[param, n])
}

// TODO: what a nightmare
data class Line(
    @JvmField
    val begin: Position2,
    @JvmField
    val dir: Vector2,  // unit norm!
    // TODO: not sure how to render this as a JVM field
    override val length: Double,
) : PositionPath<ArcLength> {
    constructor(
        begin: Position2,
        end: Position2,
    ) : this(
        begin,
        (end - begin),
        // TODO: duplication
        (end - begin).norm(),
    )

    override fun get(param: Double, n: Int) =
        DualNum.variable<ArcLength>(param, n) * dir + begin
}

class ArcLength

// TODO: this is a poor name
/**
 * @usesMathJax
 *
 * \(a^2 + b^2 = c^2\)
 */
data class ArcCurve2 @JvmOverloads constructor(
    @JvmField
    val curve: PositionPath<Internal>,
    @JvmField
    val samples: IntegralScanResult = integralScan(0.0, curve.length, 1e-6) {
        curve[it, 2].free().drop(1).norm().value()
    },
) : PositionPath<ArcLength> {
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

data class CompositePositionPath<Param> @JvmOverloads constructor(
    @JvmField
    val paths: PersistentList<PositionPath<Param>>,
    @JvmField
    val offsets: PersistentList<Double> = paths.scan(0.0) { acc, path -> acc + path.length }.toPersistentList(),
) : PositionPath<Param> {
    override val length = offsets.last()

    override fun get(param: Double, n: Int): Position2Dual<Param> {
        if (param > length) {
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
    @JvmField
    val path: PositionPath<Param>,
    @JvmField
    val offset: Double,
    override val length: Double,
) : PositionPath<Param> {
    override fun get(param: Double, n: Int) = path[param + offset, n]
}

interface HeadingPath {
    val length: Double
    operator fun get(s: Double, n: Int): Rotation2Dual<ArcLength>

    fun begin(n: Int) = get(0.0, n)
    fun end(n: Int) = get(length, n)
}

data class ConstantHeadingPath(
    @JvmField
    val heading: Rotation2,
    override val length: Double,
) : HeadingPath {
    override fun get(s: Double, n: Int) = Rotation2Dual.constant<ArcLength>(heading, n)
}

data class LinearHeadingPath(
    @JvmField
    val begin: Rotation2,
    @JvmField
    val angle: Double,
    override val length: Double,
) : HeadingPath {
    override fun get(s: Double, n: Int) =
        Rotation2Dual.exp(DualNum.variable<ArcLength>(s, n) / length * angle) * begin
}

data class SplineHeadingPath(
    // TODO: should these b fields? probably not
    val begin: Rotation2Dual<ArcLength>,
    val end: Rotation2Dual<ArcLength>,
    override val length: Double,
) : HeadingPath {
    init {
        require(begin.size >= 3)
        require(end.size >= 3)
    }

    // TODO: cleanup reparam
    // make in ctor?
    val spline =
        // s(t) = t * len
        (DualNum.variable<Internal>(1.0, 3) * length).let { s ->
            QuinticSpline1(
                begin.log().drop(1).addFirst(0.0).reparam(s),
                end.log().drop(1).addFirst(end.value() - begin.value()).reparam(s),
            )
        }

    override fun get(s: Double, n: Int) =
        // TODO: Lie plus sugar?
        Rotation2Dual.exp(
            spline[s / length, n]
                .reparam(
                    // t(s) = s / len
                    DualNum.variable<ArcLength>(s, n) / length
                )
        ) * begin.value()
}

interface PosePath {
    val length: Double
    operator fun get(s: Double, n: Int): Transform2Dual<ArcLength>

    fun begin(n: Int) = get(0.0, n)
    fun end(n: Int) = get(length, n)
}

data class TangentPath(
    @JvmField
    val path: PositionPath<ArcLength>,
    @JvmField
    val offset: Double
) : PosePath {
    override val length get() = path.length

    // NOTE: n+1 guarantees enough derivatives for tangent
    override operator fun get(s: Double, n: Int) = path[s, n + 1].let {
        Transform2Dual(it.free(), it.tangent() + offset)
    }
}

data class HeadingPosePath(
    @JvmField
    val posPath: PositionPath<ArcLength>,
    @JvmField
    val headingPath: HeadingPath,
) : PosePath {
    override val length get() = posPath.length

    init {
        require(posPath.length == headingPath.length)
    }

    override fun get(s: Double, n: Int) =
        Transform2Dual(posPath[s, n].free(), headingPath[s, n])
}

data class CompositePosePath(
    @JvmField
    val paths: PersistentList<PosePath>,
    @JvmField
    val offsets: PersistentList<Double> = paths.scan(0.0) { acc, path -> acc + path.length }.toPersistentList(),
) : PosePath {
    override val length = offsets.last()

    override fun get(s: Double, n: Int): Transform2Dual<ArcLength> {
        if (s > length) {
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
