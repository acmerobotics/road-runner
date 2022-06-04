package com.acmerobotics.roadrunner


// TODO: time is explicit!

class GvfFollower(
    val kN: Double,
    val kOmega: Double,
    val errorMapFunc: (Double) -> Double = { it },
) {

}

// do we need an angle pid controller?

class RamseteFollower(
    val b: Double,
    val zeta: Double,
)

// TODO?
// can this be turned into a trajectory follower?
class FrenetFollower(

)

// TODO: decouple the mechanism for getting the reference from the feedback
// that way you can make a holonomic path, trajectory follower from the same guts

//class PvFollower(
//    val kP: Double,
//    val kD: Double
//)

class HolonomicPidFollower()

class Feedforward(
    val kS: Double,
    val kV: Double,
    val kA: Double,
) {
    fun compute(vel: Double, accel: Double): Double {
        val basePower = vel * kV + accel * kA
        return 0.0
//        return if (basePower epsilonEquals 0.0) {
//            0.0
//        } else {
//            basePower + sign(basePower) * kS
//        }
    }
}
