package com.acmerobotics.roadrunner

import org.knowm.xchart.XYChartBuilder
import org.knowm.xchart.style.markers.Circle
import org.knowm.xchart.style.theme.MatlabTheme


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
                error.transError * 0.5,
                error.rotError * 0.01,
        )

        val velocity = (targetPose.velocity() + correction).constant()

        val dt = 0.01
        pose += Twist2Incr(
                velocity.transVel * dt,
                velocity.rotVel * dt,
        )
    }

    val chart = XYChartBuilder()
        .width(600).height(500)
        .title("Demo").xAxisTitle("x").yAxisTitle("y")
        .build()
    chart.styler.theme = MatlabTheme()

    chart.addSeries("Target",
            targets.map { it.translation.x }.toDoubleArray(),
            targets.map { it.translation.y }.toDoubleArray(),
    ).let {
        it.marker = Circle()
        it.lineWidth = 0.0f
    }

    chart.addSeries("Actual",
            measured.map { it.translation.x }.toDoubleArray(),
            measured.map { it.translation.y }.toDoubleArray(),
    ).let {
        it.marker = Circle()
        it.lineWidth = 0.0f
    }

    saveChart("demo", chart)
}
