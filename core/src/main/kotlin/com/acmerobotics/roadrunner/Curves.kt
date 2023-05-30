@file:JvmName("Curves")

package com.acmerobotics.roadrunner

/**
 * Generic, internal parameter for [DualNum].
 */
class Internal

/**
 * @usesMathJax
 *
 * Quintic spline with equation \(a t^5 + b t^4 + c t^3 + d t^2 + e t + f\) where \(0 \leq t \leq 1\)
 *
 * @property[a] \(a\)
 * @property[b] \(b\)
 * @property[c] \(c\)
 * @property[d] \(d\)
 * @property[e] \(e\)
 * @property[f] \(f\)
 */
data class QuinticSpline1d(
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

    /**
     * Fits a spline from [begin] at \(t = 0\) to [end] at \(t = 1\).
     */
    constructor(
        begin: DualNum<Internal>,
        end: DualNum<Internal>,
    ) : this(
        require(begin.size() >= 3).let {
            require(end.size() >= 3).let {
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

    /**
     * @usesMathJax
     *
     * @param[t] \(t\)
     */
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

/**
 * @usesMathJax
 *
 * Path \((x(t), y(t))\)
 *
 * @param[Param] \(t\)
 */
interface PositionPath<Param> {

    /**
     * @usesMathJax
     *
     * @param[param] \(t\)
     */
    operator fun get(param: Double, n: Int): Vector2dDual<Param>

    fun length(): Double

    fun begin(n: Int) = get(0.0, n)
    fun end(n: Int) = get(length(), n)
}

/**
 * Path comprised of two [QuinticSpline1d]s.
 */
data class QuinticSpline2d(
    @JvmField
    val x: QuinticSpline1d,
    @JvmField
    val y: QuinticSpline1d,
) : PositionPath<Internal> {
    override fun get(param: Double, n: Int) = Vector2dDual(x[param, n], y[param, n])

    override fun length() = 1.0
}

/**
 * Arclength parameter for [DualNum].
 */
class Arclength

/**
 * Line beginning at position [begin], pointed in direction [dir], and having length [length].
 *
 * @param[dir] unit vector
 */
data class Line(
    @JvmField
    val begin: Vector2d,
    @JvmField
    val dir: Vector2d,
    @JvmField
    val length: Double,
) : PositionPath<Arclength> {

    /**
     * Makes line connecting [begin] to [end].
     */
    constructor(
        begin: Vector2d,
        end: Vector2d,
    ) : this(
        begin,
        (end - begin).let { diff ->
            val norm = diff.norm()
            if (norm < 1e-6) {
                Vector2d(1.0, 0.0)
            } else {
                diff / norm
            }
        },
        (end - begin).norm(),
    )

    override fun get(param: Double, n: Int) =
        DualNum.variable<Arclength>(param, n) * dir + begin

    override fun length() = length
}

/**
 * Arclength reparameterization of [curve].
 */
data class ArclengthReparamCurve2d(
    @JvmField
    val curve: PositionPath<Internal>,
    @JvmField
    val samples: IntegralScanResult,
) : PositionPath<Arclength> {

    /**
     * @param[eps] desired error in the approximate length [length]
     */
    constructor(
        curve: PositionPath<Internal>,
        eps: Double,
    ) : this(
        curve,
        integralScan(0.0, curve.length(), eps) {
            curve[it, 2].drop(1).value().norm()
        },
    )

    @JvmField
    val length = samples.sums.last()

    fun reparam(s: Double): Double {
        return lerpLookup(samples.sums, samples.values, s)
    }

    override fun get(param: Double, n: Int): Vector2dDual<Arclength> {
        val t = reparam(param)
        val point = curve[t, n]

        val tValues = DoubleArray(n)
        tValues[0] = t
        if (n <= 1) return point.reparam(DualNum(tValues))

        val tDerivs = point.drop(1).norm().recip()
        tValues[1] = tDerivs[0]
        if (n <= 2) return point.reparam(DualNum(tValues))

        tValues[2] = tDerivs.reparam(DualNum<Arclength>(tValues))[1]
        if (n <= 3) return point.reparam(DualNum(tValues))

        tValues[3] = tDerivs.reparam(DualNum<Arclength>(tValues))[2]
        return point.reparam(DualNum(tValues))
    }

    override fun length() = length
}

data class CompositePositionPath<Param> @JvmOverloads constructor(
    @JvmField
    val paths: List<PositionPath<Param>>,
    @JvmField
    val offsets: List<Double> = paths.scan(0.0) { acc, path -> acc + path.length() },
) : PositionPath<Param> {
    @JvmField
    val length = offsets.last()

    init {
        require(paths.isNotEmpty())
    }

    override fun get(param: Double, n: Int): Vector2dDual<Param> {
        if (param > length) {
            return Vector2dDual.constant(paths.last().end(1).value(), n)
        }

        for ((offset, path) in offsets.zip(paths).reversed()) {
            if (param >= offset) {
                return path[param - offset, n]
            }
        }

        return Vector2dDual.constant(paths.first()[0.0, 1].value(), n)
    }

    override fun length() = length
}

data class PositionPathView<Param>(
    @JvmField
    val path: PositionPath<Param>,
    @JvmField
    val offset: Double,
    @JvmField
    val length: Double,
) : PositionPath<Param> {
    override fun get(param: Double, n: Int) = path[param + offset, n]

    override fun length() = length
}

/**
 * @usesMathJax
 *
 * Path heading \(\theta(s)\)
 */
interface HeadingPath {
    operator fun get(s: Double, n: Int): Rotation2dDual<Arclength>

    fun length(): Double

    fun begin(n: Int) = get(0.0, n)
    fun end(n: Int) = get(length(), n)
}

data class ConstantHeadingPath(
    @JvmField
    val heading: Rotation2d,
    @JvmField
    val length: Double,
) : HeadingPath {
    override fun get(s: Double, n: Int) = Rotation2dDual.constant<Arclength>(heading, n)

    override fun length() = length
}

data class LinearHeadingPath(
    @JvmField
    val begin: Rotation2d,
    @JvmField
    val angle: Double,
    @JvmField
    val length: Double,
) : HeadingPath {
    override fun get(s: Double, n: Int) =
        Rotation2dDual.exp(DualNum.variable<Arclength>(s, n) / length * angle) * begin

    override fun length() = length
}

data class SplineHeadingPath(
    @JvmField
    val begin: Rotation2d,
    @JvmField
    val spline: QuinticSpline1d,
    @JvmField
    val length: Double,
) : HeadingPath {
    constructor(
        begin: Rotation2dDual<Arclength>,
        end: Rotation2dDual<Arclength>,
        length: Double,
    ) : this(
        begin.value(),
        require(begin.size() >= 3).let {
            require(end.size() >= 3).let {
                // s(t) = t * len
                (DualNum.variable<Internal>(1.0, 3) * length).let { s ->
                    QuinticSpline1d(
                        DualNum.cons(0.0, begin.velocity()).reparam(s),
                        DualNum.cons(end.value() - begin.value(), end.velocity()).reparam(s),
                    )
                }
            }
        },
        length
    )

    override fun get(s: Double, n: Int) =
        Rotation2dDual.exp(
            spline[s / length, n]
                .reparam(
                    // t(s) = s / len
                    DualNum.variable<Arclength>(s, n) / length
                )
        ) * begin

    override fun length() = length
}

/**
 * @usesMathJax
 *
 * Pose path \((x(s), y(s), \theta(s))\)
 */
interface PosePath {
    operator fun get(s: Double, n: Int): Pose2dDual<Arclength>

    fun length(): Double

    fun begin(n: Int) = get(0.0, n)
    fun end(n: Int) = get(length(), n)
}

data class TangentPath(
    @JvmField
    val path: PositionPath<Arclength>,
    @JvmField
    val offset: Double
) : PosePath {
    // NOTE: n+1 guarantees enough derivatives for tangent
    override operator fun get(s: Double, n: Int) = path[s, n + 1].let {
        Pose2dDual(it, it.drop(1).angleCast() + offset)
    }

    override fun length() = path.length()
}

data class HeadingPosePath(
    @JvmField
    val posPath: PositionPath<Arclength>,
    @JvmField
    val headingPath: HeadingPath,
) : PosePath {
    init {
        require(posPath.length() == headingPath.length())
    }

    override fun get(s: Double, n: Int) =
        Pose2dDual(posPath[s, n], headingPath[s, n])

    override fun length() = posPath.length()
}

data class CompositePosePath(
    @JvmField
    val paths: List<PosePath>,
    @JvmField
    val offsets: List<Double> = paths.scan(0.0) { acc, path -> acc + path.length() },
) : PosePath {
    init {
        require(paths.size + 1 == offsets.size)
    }

    @JvmField
    val length = offsets.last()

    override fun get(s: Double, n: Int): Pose2dDual<Arclength> {
        if (s > length) {
            return Pose2dDual.constant(paths.last().end(1).value(), n)
        }

        for ((offset, path) in offsets.zip(paths).reversed()) {
            if (s >= offset) {
                return path[s - offset, n]
            }
        }

        return Pose2dDual.constant(paths.first()[0.0, 1].value(), n)
    }

    override fun length() = length
}

/**
 * Project position [query] onto position path [path] starting with initial guess [init].
 */
fun project(path: PositionPath<Arclength>, query: Vector2d, init: Double): Double {
    // TODO: is ten iterations enough?
    return (1..10).fold(init) { s, _ ->
        val guess = path[s, 3]
        val ds = (query - guess.value()) dot guess.drop(1).value()
        clamp(s + ds, 0.0, path.length())
    }
}

/**
 * Project position [query] onto position path [path] starting with initial guess [init].
 */
fun project(path: PosePath, query: Vector2d, init: Double): Double {
    // TODO: is ten iterations enough?
    return (1..10).fold(init) { s, _ ->
        val guess = path[s, 3].position.bind()
        val ds = (query - guess.value()) dot guess.drop(1).value()
        clamp(s + ds, 0.0, path.length())
    }
}
