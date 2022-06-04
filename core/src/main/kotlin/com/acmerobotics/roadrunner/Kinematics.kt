package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.trajectory.constraints.UnsatisfiableConstraint
import kotlin.math.abs

//fun fieldToRobotVelocity(fieldPose: Pose2d, fieldVel: Pose2d) =
//    Pose2d(fieldVel.vec().rotated(-fieldPose.heading), fieldVel.heading)

// TODO: should there be another type for velocities?
class WheelVelocities<N : Num<N>>(
    val frontLeft: N,
    val frontRight: N,
    val backLeft: N,
    val backRight: N,
) {
    // TODO: necessary?
    fun toList() = listOf(frontLeft, frontRight, backLeft, backRight)
}

// TODO: I'm probably going to monomorphize to remove Num<N>; this is a step in that direction
class WheelIncrements(
        val frontLeft: Double,
        val frontRight: Double,
        val backLeft: Double,
        val backRight: Double,
)

// TODO: how to integrate this with constraints?
class MecanumKinematics @JvmOverloads constructor(
    val trackWidth: Double,
    val lateralMultiplier: Double = 1.0
) {
    constructor(
        trackWidth: Double, wheelBase: Double,
        lateralMultiplier: Double = 1.0
    ): this((trackWidth + wheelBase) / 2, lateralMultiplier)

    fun <N : Num<N>> forward(w: WheelVelocities<N>) = Twist2(
        (w.backRight + w.frontRight - w.frontLeft - w.backLeft) * (0.25 / trackWidth),
        Vector2(
            (w.backLeft + w.frontRight + w.backRight + w.frontLeft) * 0.25,
            (w.backLeft + w.frontRight - w.frontLeft - w.backRight) * (0.25 / lateralMultiplier),
        )
    )

    // TODO: what about accel? maybe linear kinematics should be good?
    // TODO: is twist the right type here?
    fun <N : Num<N>> inverse(t: Twist2<N>) = WheelVelocities(
            t.translation.x - t.translation.y * lateralMultiplier - t.rotation * trackWidth,
            t.translation.x + t.translation.y * lateralMultiplier - t.rotation * trackWidth,
            t.translation.x - t.translation.y * lateralMultiplier + t.rotation * trackWidth,
            t.translation.x + t.translation.y * lateralMultiplier + t.rotation * trackWidth,
    )

}

// TODO: should this be an inner class?
// TODO: the constraint should support upper and lower bounds, right?
// is there a way to save computation? well I suppose the overlap is minimized by only computing the pose and its derivatives once

// TODO: test end-to-end

// TODO: is there a way to generalize all of the kinematics stuff?
// they're all just linear at the end of the day (but I don't want any dependencies)
class MecanumVelConstraint(val kinematics: MecanumKinematics, val maxWheelVel: Double) {
    fun maxVelocity(maxWheelVel: Double, path: PosePath, s: Double, baseRobotVel: Twist2<DoubleNum>): Double {
        val baseWheelVels = kinematics.inverse(baseRobotVel)
        if (baseWheelVels.toList().maxOf { abs(it.value) } >= maxWheelVel) {
            throw UnsatisfiableConstraint()
        }

        //        pose: Pose2d, deriv: Pose2d,

        val pose = path[s, 2]
        // pose is txWorldRobot, deriv is world
        val robotDeriv = pose.constant().inverse() * pose.dropOne().constant()

        val wheelDerivs = kinematics.inverse(robotDeriv)
        return baseWheelVels.zip(wheelDerivs).map {
            max(
                    (maxWheelVel - it.first) / it.second,
                    (-maxWheelVel - it.first) / it.second
            )
        }.minOrNull()!!
    }
}

// TODO: is a generic tracking wheel localizer worth it?
// is it still feasible with Cramer's rule?
