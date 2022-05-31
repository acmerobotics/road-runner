package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.trajectory.constraints.UnsatisfiableConstraint
import kotlin.math.abs

//fun fieldToRobotVelocity(fieldPose: Pose2d, fieldVel: Pose2d) =
//    Pose2d(fieldVel.vec().rotated(-fieldPose.heading), fieldVel.heading)

class WheelDeltas<N : Num<N>>(
    val frontLeft: N,
    val frontRight: N,
    val backLeft: N,
    val backRight: N,
) {
    // TODO: necessary?
    fun toList() = listOf(frontLeft, frontRight, backLeft, backRight)
}

// TODO: how to integrate this with constraints?
class MecanumKinematics @JvmOverloads constructor(
    val trackWidth: Double,
    val lateralMultiplier: Double = 1.0
) {
    constructor(
        trackWidth: Double, wheelBase: Double,
        lateralMultiplier: Double = 1.0
    ): this((trackWidth + wheelBase) / 2, lateralMultiplier)

    fun <N : Num<N>> forward(w: WheelDeltas<N>) = Twist2(
        (w.backRight + w.frontRight - w.frontLeft - w.backLeft) * (0.25 / trackWidth),
        Vector2(
            (w.backLeft + w.frontRight + w.backRight + w.frontLeft) * 0.25,
            (w.backLeft + w.frontRight - w.frontLeft - w.backRight) * (0.25 / lateralMultiplier),
        )
    )

    // TODO: what about accel? maybe linear kinematics should be good?
    fun <N : Num<N>> inverse(t: Twist2<N>) = WheelDeltas(
        t.x - t.y * lateralMultiplier - t.theta * trackWidth,
        t.x + t.y * lateralMultiplier - t.theta * trackWidth,
        t.x - t.y * lateralMultiplier + t.theta * trackWidth,
        t.x + t.y * lateralMultiplier + t.theta * trackWidth,
    )

    fun maxVelocity(maxWheelVel: Double, path: PosePath, s: Double, baseRobotVel: Twist2<DoubleNum>): Double {
        val baseWheelVels = inverse(baseRobotVel)
        if (baseWheelVels.toList().maxOf { abs(it.value) } >= maxWheelVel) {
            throw UnsatisfiableConstraint()
        }

//        pose: Pose2d, deriv: Pose2d,

        val pose = path[s, 2]
        // pose is txWorldRobot, deriv is world
        val robotDeriv = pose.constant().inverse() * pose.dropOne().constant()

        val wheelDerivs = inverse(robotDeriv)
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
