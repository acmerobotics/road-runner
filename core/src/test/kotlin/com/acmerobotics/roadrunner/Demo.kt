package com.acmerobotics.roadrunner

import kotlin.math.PI
import kotlin.math.cos

fun main() {
    val path = TangentPath(ArcCurve2(
        QuinticSpline2(
            QuinticSpline1(
                DualNum(doubleArrayOf(0.0, 40.0, 0.0)),
                DualNum(doubleArrayOf(80.0, 40.0, 0.0)),
            ),
            QuinticSpline1(
                DualNum(doubleArrayOf(0.0, 0.0, 0.0)),
                DualNum(doubleArrayOf(20.0, 0.0, 0.0)),
            ),
        )
    ), Rotation2.exp(0.0))

    val maxVel = 1.0
    val profile = Profile(
            listOf(
                    Profile.ConstVelSegment(
                            DualNum.constant(0.0, 4),
                            maxVel, path.length / maxVel
                    )
            )
    )

    val trajectory = Trajectory(path, profile)

    var s = 0.0
//    var pose = Transform2Dual<Time>(
//            Vector2Dual(DualNum.constant(1.0, 3), DualNum.constant(-3.0, 3)),
//            Rotation2Dual.exp(DualNum.constant(cos(PI / 8), 3)),
//    ) * trajectory[0.0, 3]
    var pose = Transform2Dual<Time>(
        Vector2Dual(
            DualNum.constant(5.0, 3),
            DualNum.constant(5.0, 3),
        ),
        Rotation2Dual.exp(DualNum.constant(0.0, 3)),
    )

    val measured = mutableListOf<Transform2>()
    val targets = mutableListOf<Transform2>()

    while (true) {
        s = trajectory.project(pose.translation.bind(), s).constant()

        if (s >= path.length - 0.25) {
            break
        }

        val targetPose = trajectory.getByDisp(s, 3)

        measured.add(pose.constant())
        targets.add(targetPose.constant())

        val error = localError(targetPose.constant(), pose.constant())
        val correction = Twist2(
                error.transError * 1.0,
                error.rotError * 1.0,
        )

        val velocity = (targetPose.velocity() + correction).constant()

        val dt = 0.01
        pose += Twist2Incr(
                velocity.transVel * dt,
                velocity.rotVel * dt,
        )
    }
}
