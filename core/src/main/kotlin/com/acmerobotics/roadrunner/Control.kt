package com.acmerobotics.roadrunner

import kotlin.math.withSign

data class Feedforward(
    @JvmField
    val kS: Double,
    @JvmField
    val kV: Double,
    @JvmField
    val kA: Double,
) {
    // TODO: choose signature
//    fun compute(vel: DualNum<Time>): Double {
    fun compute(vel: Double, accel: Double): Double {
        val basePower = vel * kV + accel * kA
        return if (basePower == 0.0) {
            0.0
        } else {
            basePower + kS.withSign(basePower)
        }
    }
}

data class Transform2Error(@JvmField val transError: Vector2, @JvmField val rotError: Double)

// NOTE: SE(2) minus mixes the frame orientations, and we need it purely in the actual frame
// TODO: does this need its own type?
// TODO: naming?
fun poseError(targetPose: Transform2, actualPose: Transform2): Transform2Error {
    val transErrorWorld = targetPose.translation - actualPose.translation
    val rotError = targetPose.rotation - actualPose.rotation
    return Transform2Error(actualPose.rotation.inverse() * transErrorWorld, rotError)
}

class HolonomicController(
    val posGain: Double,
    val velGain: Double,
    val headingGain: Double,
    val headingVelGain: Double,
) {
    fun compute(
        targetPose: Transform2Dual<Time>,
        actualPose: Transform2,
        actualBodyVel: Twist2,
    ): Twist2Dual<Time> {
        val poseError = poseError(targetPose.value(), actualPose)

        val targetVel = targetPose.velocity()
        val velError = targetVel.value() - actualBodyVel

        // TODO: is inverseThenTimes() better than the alternative?
        return targetPose.rotation.inverse() * targetVel +
            Twist2(
                poseError.transError * posGain + velError.transVel * velGain,
                poseError.rotError * headingGain + velError.rotVel * headingVelGain,
            )
    }
}
