package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.markers.None
import org.knowm.xchart.style.theme.MatlabTheme
import kotlin.math.PI

class RamseteTest {
    @Test
    fun testRamsete() {
        testRamseteHelper(false)
        testRamseteHelper(true)
    }

    fun testRamseteHelper(reversed: Boolean) {
        val path = TangentPath(
            PositionPathSeqBuilder(
                Vector2d(0.0, 0.0),
                0.0,
                1e-6
            )
                .splineTo(
                    Vector2d(30.0, 30.0),
                    PI / 4
                )
                .lineToY(60.0)
                .splineTo(
                    Vector2d(60.0, 90.0),
                    PI,
                )
                .build()
                .first(),
            if (reversed) PI else 0.0
        )

        val trackWidth = 15.0
        val kinematics = TankKinematics(trackWidth)

        val profile = TimeProfile(
            profile(
                TEST_PROFILE_PARAMS,
                path, 0.0,
                kinematics.WheelVelConstraint(10.0),
                ProfileAccelConstraint(-20.0, 20.0),
            ).baseProfile
        )

        val controller = RamseteController(trackWidth, bBar = 2.0)

        // var pose = (
        //     Vector2(-1.0, -1.0),
        //     PI / 8
        // )

        var pose = Pose2d(
            Vector2d(-5.0, -10.0),
            if (reversed) PI else 0.0
        )

        val targetPoses = mutableListOf<Pose2d>()
        val poses = mutableListOf(pose)

        val dt = 0.01
        var t = 0.0
        while (t < profile.duration) {
            val s = profile[t]

            val targetPose = path[s.value(), 3]

            val command = controller.compute(s, targetPose, pose).value()

            pose +=
                Twist2d(
                    command.linearVel * dt,
                    command.angVel * dt
                )

            targetPoses.add(targetPose.value())
            poses.add(pose)

            t += dt
        }

        val graph = XYChart(600, 400)
        graph.title = "Ramsete Follower Sim"
        graph.addSeries(
            "Target Trajectory",
            targetPoses.map { it.position.x }.toDoubleArray(),
            targetPoses.map { it.position.y }.toDoubleArray()
        )
        graph.addSeries(
            "Actual Trajectory",
            poses.map { it.position.x }.toDoubleArray(),
            poses.map { it.position.y }.toDoubleArray()
        )
        graph.seriesMap.values.forEach { it.marker = None() }
        graph.styler.theme = MatlabTheme()
        saveChart("ramseteFollower${if (reversed) "Reversed" else ""}", graph)
    }
}
