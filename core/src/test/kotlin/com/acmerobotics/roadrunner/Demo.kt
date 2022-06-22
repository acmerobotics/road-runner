package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import org.knowm.xchart.XYChartBuilder
import org.knowm.xchart.style.markers.Circle
import org.knowm.xchart.style.theme.MatlabTheme

class Demo {
    @Test
    fun test() {
        val path = TangentPath(
            ArclengthReparamCurve2(
                QuinticSpline2(
                    QuinticSpline1(
                        DualNum(doubleArrayOf(0.0, 40.0, 0.0)),
                        DualNum(doubleArrayOf(80.0, 40.0, 0.0)),
                    ),
                    QuinticSpline1(
                        DualNum(doubleArrayOf(0.0, 0.0, 0.0)),
                        DualNum(doubleArrayOf(20.0, 0.0, 0.0)),
                    ),
                ),
                1e-6,
            ),
            0.0
        )

        val profile = DisplacementProfile(
            listOf(0.0, path.length()),
            listOf(1.0, 1.0),
            listOf(0.0)
        )

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
            s = project(path, pose.trans.value().bind(), s)

            if (s >= path.length() - 0.25) {
                break
            }

            val targetPose = path[s, 3].reparam(profile[s])

            measured.add(pose.value())
            targets.add(targetPose.value())

            val error = targetPose.value().minusExp(pose.value())
            val correction = Twist2(
                error.trans * 0.5,
                error.rot.log() * 0.01,
            )

            val velocity = (targetPose.velocity() + correction).value()

            val dt = 0.01
            pose += Twist2Increment(
                velocity.transVel * dt,
                velocity.rotVel * dt,
            )
        }

        val chart = XYChartBuilder()
            .width(600).height(500)
            .title("Demo").xAxisTitle("x").yAxisTitle("y")
            .build()
        chart.styler.theme = MatlabTheme()

        chart.addSeries(
            "Target",
            targets.map { it.trans.x }.toDoubleArray(),
            targets.map { it.trans.y }.toDoubleArray(),
        ).let {
            it.marker = Circle()
            it.lineWidth = 0.0f
        }

        chart.addSeries(
            "Actual",
            measured.map { it.trans.x }.toDoubleArray(),
            measured.map { it.trans.y }.toDoubleArray(),
        ).let {
            it.marker = Circle()
            it.lineWidth = 0.0f
        }

        saveChart("demo", chart)
    }
}
