package com.acmerobotics.roadrunner

import kotlin.math.PI
import kotlin.math.cos

fun main() {
    val path = TangentPath(ArcCurve2(
            QuinticSpline2(
                    QuinticSpline1(
                            DualNum(doubleArrayOf(0.0, 0.0, 30.0)),
                            DualNum(doubleArrayOf(20.0, 30.0, 0.0)),
                    ),
                    QuinticSpline1(
                            DualNum(doubleArrayOf(0.0, 0.0, 10.0)),
                            DualNum(doubleArrayOf(20.0, 15.0, 0.0)),
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
    var pose = Transform2Dual<Time>(
            Vector2Dual(DualNum.constant(1.0, 3), DualNum.constant(-3.0, 3)),
            Rotation2Dual.exp(DualNum.constant(cos(PI / 8), 3)),
    ) * trajectory[0.0, 3]

    val measured = mutableListOf<Transform2>()
    val targets = mutableListOf<Transform2>()

    while (true) {
        s = trajectory.project(pose.translation.bind(), s).constant()

        println(s)

        if (s >= path.length - 0.25) {
            break
        }

        val targetPose = trajectory.getByDisp(s, 3)

        measured.add(pose.constant())
        targets.add(targetPose.constant())

        val error = targetPose.constant() - pose.constant()
        val correction = Twist2(
                error.transIncr * 10.0,
                error.rotIncr * 0.01,
        )

        val dt = 0.01
        pose += Twist2Incr(
                correction.transVel * dt,
                correction.rotVel * dt,
        )
    }
}
