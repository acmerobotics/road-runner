@file:JvmName("Trajectory")

package com.acmerobotics.roadrunner

// TODO: projection should not be compositional
// either we destroy CompositePosePath and change the interface
// or we duplicate the projection code for PosePath and PositionPath<ArcLength>

// keeping PosePath abstract makes reflected/transformed paths easier

// TODO: do we even need this class?
// I'm less and less happy with its existence
data class DisplacementTrajectory(
    @JvmField
    val path: PosePath,
    @JvmField
    val dispProfile: DisplacementProfile,
) {
    operator fun get(s: Double, n: Int) = path[s, n].reparam(dispProfile[s])

    fun project(query: Position2Dual<Time>, init: Double) =
        project(path, query.value(), init).let { s ->
            val r = path[s, 3].translation
                .bind()
                .reparam(dispProfile[s])

            val d = query - r
            val drds = r.tangentVec()
            val d2rds2 = drds.drop(1)

            val dsdt = (query.tangentVec() dot drds) / ((d dot d2rds2) - 1.0)

            dsdt.addFirst(s)
        }
}

data class TimeTrajectory @JvmOverloads constructor(
    @JvmField
    val dispTrajectory: DisplacementTrajectory,
    @JvmField
    val timeProfile: TimeProfile = TimeProfile(dispTrajectory.dispProfile),
) {
    operator fun get(t: Double, n: Int) = timeProfile[t].let { s ->
        dispTrajectory.path[s.value(), n].reparam(s)
    }

    // TODO: are either of these methods necessary?
    // they create pressure on other interfaces to be (Displacement|Time)Trajectory agnostic
    fun getByDisp(s: Double, n: Int) = dispTrajectory[s, n]
    fun project(query: Position2Dual<Time>, init: Double) = dispTrajectory.project(query, init)
}

// TODO: separate max vel/accel functions?
// pro: more efficient, more ergonomic
//   vel, accel often independent conditional on the robotPose
//   more advanced constraints are already out of algorithmic reach
// con: more allocations, unboxing (although maybe not considering Interval)
//   though then the composition constraint has to box again

fun interface VelocityConstraint {
    fun maxRobotVel(robotPose: Transform2Dual<Arclength>): Double
}

fun interface AccelerationConstraint {
    fun minMaxProfileAccel(robotPose: Transform2Dual<Arclength>): Interval
}

fun profile(
    path: PosePath,
    beginEndVel: Double,
    maxVel: VelocityConstraint,
    minMaxAccel: AccelerationConstraint,
    resolution: Double,
) = profile(
    path.length(),
    beginEndVel,
    { maxVel.maxRobotVel(path[it, 2]) },
    { minMaxAccel.minMaxProfileAccel(path[it, 2]) },
    resolution,
)

fun forwardProfile(
    path: PosePath,
    beginVel: Double,
    maxVel: VelocityConstraint,
    minMaxAccel: AccelerationConstraint,
    resolution: Double,
) = forwardProfile(
    path.length(),
    beginVel,
    { maxVel.maxRobotVel(path[it, 2]) },
    { minMaxAccel.minMaxProfileAccel(path[it, 2]).max },
    resolution,
)

fun backwardProfile(
    path: PosePath,
    maxVel: VelocityConstraint,
    endVel: Double,
    minMaxAccel: AccelerationConstraint,
    resolution: Double,
) = backwardProfile(
    path.length(),
    { maxVel.maxRobotVel(path[it, 2]) },
    endVel,
    { minMaxAccel.minMaxProfileAccel(path[it, 2]).min },
    resolution,
)
