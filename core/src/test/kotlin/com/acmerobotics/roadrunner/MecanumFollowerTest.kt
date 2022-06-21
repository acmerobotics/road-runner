package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.markers.None
import org.knowm.xchart.style.theme.MatlabTheme
import kotlin.math.PI
import kotlin.random.Random

class MecanumFollowerTest {
    @Test
    fun testTimeFollower() {
        val dt = 1 / 25.0

        val kinematics = MecanumKinematics(3.0)

        val traj = TangentPath(
            PositionPathBuilder(
                Position2(0.0, 0.0),
                Rotation2.exp(0.0),
                1e-6,
            )
                .splineTo(
                    Position2(15.0, 15.0),
                    Rotation2.exp(PI),
                )
                .splineTo(
                    Position2(5.0, 35.0),
                    Rotation2.exp(PI / 3),
                )
                .build(),
            0.0,
        ).let { path ->
            TimeTrajectory(
                DisplacementTrajectory(
                    path,
                    profile(
                        path,
                        0.0,
                        // TODO: angular velocity constraint
                        kinematics.MaxWheelVelocityConstraint(50.0),
                        AccelerationConstraint { Interval(-25.0, 25.0) },
                        // TODO: resolution
                        0.25,
                    )
                )
            )
        }

        val follower = HolonomicController(10.0, 10.0, 0.1)

        val ff = MotorFeedforward(0.0, 1 / 60.0, 0.0)

        var poseEstimate = Transform2(
            Vector2(0.0, 0.0),
            Rotation2.exp(0.0),
        )

        val targetPositions = mutableListOf<Vector2>()
        val actualPositions = mutableListOf<Vector2>()

        var t = 0.0
        while (t <= traj.timeProfile.duration) {
            t += dt

            val targetPose = traj[t, 3]

            targetPositions.add(targetPose.translation.value())
            actualPositions.add(poseEstimate.translation)

            val command = follower.compute(
                targetPose,
                poseEstimate,
                Twist2(
                    Vector2(0.0, 0.0),
                    0.0,
                )
            )

            val wheelVoltages = kinematics.inverse(command)
//                fun all() = listOf(frontLeft, frontRight, backLeft, backRight)
                .all()
                .map { ff.compute(it) }
                .map { clamp(it * (1.0 + 0.3 * (Random.Default.nextDouble() - 0.5)), -1.0, 1.0) }

            val posDeltas = wheelVoltages
                .map { 60.0 * it * dt }

            // TODO: should this be a list?
            // maybe there should just be a list secondary ctor
            val wheelIncrements = MecanumKinematics.WheelIncrements<Time>(
                DualNum.constant(posDeltas[0], 2),
                DualNum.constant(posDeltas[1], 2),
                DualNum.constant(posDeltas[2], 2),
                DualNum.constant(posDeltas[3], 2),
            )

            poseEstimate += kinematics.forward(wheelIncrements).value()
        }

        val graph = XYChart(600, 400)
        graph.title = "Mecanum PIDVA Follower Sim"
        graph.addSeries(
            "Target Trajectory",
            targetPositions.map { it.x }.toDoubleArray(),
            targetPositions.map { it.y }.toDoubleArray()
        )
        graph.addSeries(
            "Actual Trajectory",
            actualPositions.map { it.x }.toDoubleArray(),
            actualPositions.map { it.y }.toDoubleArray()
        )
        graph.seriesMap.values.forEach { it.marker = None() }
        graph.styler.theme = MatlabTheme()
        saveChart("mecanumTimeFollower", graph)
    }

    @Test
    fun testDispFollower() {
        val dt = 1 / 25.0

        val kinematics = MecanumKinematics(3.0)

        val path = TangentPath(
            PositionPathBuilder(
                Position2(0.0, 0.0),
                Rotation2.exp(0.0),
                1e-6,
            )
                .splineTo(
                    Position2(15.0, 15.0),
                    Rotation2.exp(PI),
                )
                .splineTo(
                    Position2(5.0, 35.0),
                    Rotation2.exp(PI / 3),
                )
                .build(),
            0.0,
        )

        val profile =
            profile(
                path,
                5.0,
                // TODO: angular velocity constraint
                kinematics.MaxWheelVelocityConstraint(50.0),
                { Interval(-25.0, 25.0) },
                // TODO: resolution
                0.25,
            )

        val follower = HolonomicController(10.0, 10.0, 0.1)

        val ff = MotorFeedforward(0.0, 1 / 60.0, 0.0)

        var poseEstimate = Transform2Dual.constant<Time>(
            Transform2(
                Vector2(5.0, 5.0),
                Rotation2.exp(0.0),
            ),
            2
        )

        val targetPositions = mutableListOf<Vector2>()
        val actualPositions = mutableListOf<Vector2>()

        var s = 0.0
        var i = 0
        while (s <= profile.length) {
            if (i++ >= 25) {
                break
            }

            s = project(path, poseEstimate.translation.value().bind(), s)

            val targetPose = path[s, 3]

            val rshort = targetPose.translation.reparam(profile[s])
            val r = Vector2Dual<Time>(
                DualNum(doubleArrayOf(rshort.x[0], rshort.x[1], rshort.x[2], 0.0)),
                DualNum(doubleArrayOf(rshort.y[0], rshort.y[1], rshort.y[2], 0.0))
            )
            val d = poseEstimate.translation - r
            val drds = r.drop(1)
            val d2rds2 = drds.drop(1)

            val dsdt = (
                Vector2Dual<Time>(
                    DualNum(doubleArrayOf(poseEstimate.translation.x[1], 0.0)),
                    DualNum(doubleArrayOf(poseEstimate.translation.y[1], 0.0)),
                )
                    dot drds
                ) / ((d dot d2rds2) - 1.0)

            val sDual = dsdt.addFirst(s)
            // val sDual = DualNum<Time>(doubleArrayOf(s, dsdt[0], 0.0))

            // println(poseEstimate)

            // TODO: sDual[2] is zero with no profile jerk and no robot acceleration
            // it becomes nonzero with just robot accel
            // profile jerk alone does nothing (but a zero is necessary for indexing)

            println(sDual[2])

            targetPositions.add(targetPose.translation.value())
            actualPositions.add(poseEstimate.translation.value())

            val command = follower.compute(
                targetPose.reparam(sDual),
                poseEstimate.value(),
                Twist2(
                    Vector2(0.0, 0.0),
                    0.0,
                )
            )

            val wheelVoltages = kinematics.inverse(command)
//                fun all() = listOf(frontLeft, frontRight, backLeft, backRight)
                .all()
                .map { ff.compute(it) }
                .map { clamp(it * (1.0 + 0.3 * (Random.Default.nextDouble() - 0.5)), -1.0, 1.0) }

            val posDeltas = wheelVoltages
                .map { 60.0 * it * dt }

            // TODO: should this be a list?
            // maybe there should just be a list secondary ctor
            val wheelIncrements = MecanumKinematics.WheelIncrements<Time>(
                DualNum.constant(posDeltas[0], 2),
                DualNum.constant(posDeltas[1], 2),
                DualNum.constant(posDeltas[2], 2),
                DualNum.constant(posDeltas[3], 2),
            )

            poseEstimate += kinematics.forward(wheelIncrements)
        }

        val graph = XYChart(600, 400)
        graph.title = "Mecanum PIDVA Follower Sim"
        graph.addSeries(
            "Target Trajectory",
            targetPositions.map { it.x }.toDoubleArray(),
            targetPositions.map { it.y }.toDoubleArray()
        )
        graph.addSeries(
            "Actual Trajectory",
            actualPositions.map { it.x }.toDoubleArray(),
            actualPositions.map { it.y }.toDoubleArray()
        )
        graph.seriesMap.values.forEach { it.marker = None() }
        graph.styler.theme = MatlabTheme()
        saveChart("mecanumDispFollower", graph)
    }
}
