package com.acmerobotics.roadrunner

// TODO: separate or equal? getByDisp() is awkward but nice to unify profiles and trajectories
class DisplacementTrajectory(
    val path: PosePath,
    val dispProfile: DisplacementProfile,
) {
    operator fun get(s: Double, n: Int) = path[s, n].reparam(dispProfile[s])

    fun project(query: Position2Dual<Time>, init: Double) =
        project(path, query.constant(), init).let { s ->
            val r = path[s, 3].translation
                .bind()
                .reparam(dispProfile[s])

            val d = query - r
            val drds = r.tangentVec()
            val d2rds2 = drds.drop(1)

            val dsdt = (query.tangentVec() dot drds) * ((d dot d2rds2) + (-1.0)).recip()

            // TODO: "Time" here should be inferred... we might need a method
            DualNum<Time>(doubleArrayOf(s) + dsdt.values)
        }
}

class TimeTrajectory(
    val dispTrajectory: DisplacementTrajectory,
) {
    val timeProfile = TimeProfile(dispTrajectory.dispProfile)

    operator fun get(t: Double, n: Int) = timeProfile[t].let { s ->
        dispTrajectory.path[s.constant(), n].reparam(s)
    }

    fun getByDisp(s: Double, n: Int) = dispTrajectory[s, n]

    fun project(query: Position2Dual<Time>, init: Double) = dispTrajectory.project(query, init)
}

