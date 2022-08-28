package com.acmerobotics.roadrunner

// short and non-generic for ease of use
class Trajectory(
    @JvmField
    val path: PosePath,
    @JvmField
    val profile: CancelableProfile,
    @JvmField
    val offsets: List<Double>
) {
    fun cancel(s: Double): DisplacementTrajectory {
        val offset = s
        return DisplacementTrajectory(
            object : PosePath {
                override fun length() = path.length() - offset
                override fun get(s: Double, n: Int) = path[s + offset, n]
            },
            profile.cancel(s)
        )
    }
}

class DisplacementTrajectory(
    @JvmField
    val path: PosePath,
    @JvmField
    val profile: DisplacementProfile
) {
    constructor(t: Trajectory) : this(t.path, t.profile.baseProfile)

    operator fun get(s: Double) = path[s, 3].reparam(profile[s])
}

class TimeTrajectory(
    @JvmField
    val path: PosePath,
    @JvmField
    val profile: TimeProfile
) {
    @JvmField
    val duration = profile.duration

    constructor(t: Trajectory) : this(t.path, TimeProfile(t.profile.baseProfile))

    constructor(t: DisplacementTrajectory) : this(t.path, TimeProfile(t.profile))

    operator fun get(t: Double): Transform2dDual<Time> {
        val s = profile[t]
        return path[s.value(), 3].reparam(s)
    }
}
