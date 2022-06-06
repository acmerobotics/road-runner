package com.acmerobotics.roadrunner


fun clamp(x: Double, lo: Double, hi: Double): Double {
    if (x < lo) {
        return lo
    }
    if (x > hi) {
        return hi
    }
    return x
}


class Profile(val segments: List<Segment>) {
    interface Segment {
        val duration: Double
        operator fun get(t: Double): DualNum<Time>

        val distance: Double
        fun getByDisp(s: Double): DualNum<Time>

        fun end() = get(duration)
    }

    class ConstVelSegment(
            start: DualNum<Time>,
            val vel: Double,
            override val distance: Double) : Segment {
        override val duration = distance / vel

        val startPos = start.values[0]

        val n = start.values.size

        override fun get(t: Double): DualNum<Time> {
            val t = DualNum.variable<Time>(clamp(t, 0.0, duration), n)
            return t * vel + startPos
        }

        override fun getByDisp(s: Double) =
                DualNum<Time>(doubleArrayOf(clamp(s, 0.0, distance), vel, 0.0, 0.0))
    }

    val duration = segments.sumOf { it.duration }
    operator fun get(t: Double): DualNum<Time> {
        var segmentTime = clamp(t, 0.0, duration)
        for (segment in segments) {
            if (segmentTime > segment.duration) {
                segmentTime -= segment.duration
                continue
            }
            return segment[segmentTime]
        }
        throw AssertionError()
    }

    val distance = segments.sumOf { it.distance }
    fun getByDisp(s: Double): DualNum<Time> {
        var segmentDisp = clamp(s, 0.0, distance)
        for (segment in segments) {
            if (segmentDisp > segment.distance) {
                segmentDisp -= segment.distance
                continue
            }
            return segment.getByDisp(segmentDisp)
        }
        throw AssertionError()
    }
}

class Trajectory(private val path: PosePath, private val profile: Profile) {
    val duration: Double = profile.duration

    operator fun get(t: Double, n: Int): Transform2Dual<Time> = profile[t].let { s ->
        path[s.values[0], n].reparam(s) }

    fun getByDisp(s: Double, n: Int): Transform2Dual<Time> = profile.getByDisp(s).let { s ->
        path[s.values[0], n].reparam(s) }

    fun project(query: Position2Dual<Time>, init: Double) =
            project(path, query.constant(), init).let { s ->
                val r = path[s, 3].translation
                        .bind()
                        .reparam(profile.getByDisp(s))

                val d = query - r
                val drds = r.tangentVec()
                val d2rds2 = drds.drop(1)

                val dsdt = (query.tangentVec() dot drds) * ((d dot d2rds2) + (-1.0)).recip()

                // TODO: "Time" here should be inferred... we might need a method
                DualNum<Time>(doubleArrayOf(s) + dsdt.values)
            }
}
