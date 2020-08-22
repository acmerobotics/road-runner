package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.TankDrive
import com.acmerobotics.roadrunner.followers.TankGVFFollower
import com.acmerobotics.roadrunner.followers.RamseteFollower
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.PathBuilder
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints
import com.acmerobotics.roadrunner.util.DoubleProgression
import org.apache.commons.math3.distribution.NormalDistribution
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.MatlabTheme
import org.knowm.xchart.style.markers.None
import kotlin.math.PI
import kotlin.math.max
import kotlin.math.min

private const val kV = 1.0 / 60.0
private const val SIMULATION_HZ = 25
private const val TRACK_WIDTH = 3.0

private val BASE_CONSTRAINTS = DriveConstraints(50.0, 25.0, 0.0, PI / 2, PI / 2, 0.0)
private val CONSTRAINTS = TankConstraints(BASE_CONSTRAINTS, TRACK_WIDTH)

private val VOLTAGE_NOISE_DIST = NormalDistribution(1.0, 0.05)

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class TankFollowerTest {

    private class SimulatedTankDrive(
        private val dt: Double,
        private val kV: Double,
        trackWidth: Double
    ) : TankDrive(kV, 0.0, 0.0, trackWidth) {
        override val rawExternalHeading = Double.NaN
        var powers = listOf(0.0, 0.0)
        var positions = listOf(0.0, 0.0)

        init {
            localizer = TankLocalizer(this, false)
        }

        override fun setMotorPowers(left: Double, right: Double) {
            powers = listOf(left, right)
                    .map { it * VOLTAGE_NOISE_DIST.sample() }
                    .map { max(-1.0, min(it, 1.0)) }
            positions = positions.zip(powers)
                    .map { it.first + it.second / kV * dt }
        }

        override fun getWheelPositions() = positions
    }

    @Test
    fun simulatePIDVAFollower() {
        val dt = 1.0 / SIMULATION_HZ

        val trajectory = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), constraints = CONSTRAINTS)
                .splineTo(Vector2d(15.0, 15.0), PI)
                .splineTo(Vector2d(5.0, 35.0), PI / 3)
                .build()

        val clock = SimulatedClock()
        val drive = SimulatedTankDrive(dt, kV, TRACK_WIDTH)
        val follower = TankPIDVAFollower(
                PIDCoefficients(0.01),
                PIDCoefficients(kP = 0.1, kD = 0.001),
            Pose2d(0.5, 0.5, Math.toRadians(3.0)),
                1.0,
                clock
        )
        follower.followTrajectory(trajectory)

        val targetPositions = mutableListOf<Vector2d>()
        val actualPositions = mutableListOf<Vector2d>()

        drive.poseEstimate = trajectory.start()
        var t = 0.0
        while (follower.isFollowing()) {
            t += dt
            clock.time = t
            val signal = follower.update(drive.poseEstimate)
            drive.setDriveSignal(signal)
            drive.updatePoseEstimate()

            targetPositions.add(trajectory[t].vec())
            actualPositions.add(drive.poseEstimate.vec())
        }

        val graph = XYChart(600, 400)
        graph.title = "Tank PIDVA Follower Sim (%.2fs)".format(t)
        graph.addSeries(
                "Target Trajectory",
                targetPositions.map { it.x }.toDoubleArray(),
                targetPositions.map { it.y }.toDoubleArray())
        graph.addSeries(
                "Actual Trajectory",
                actualPositions.map { it.x }.toDoubleArray(),
                actualPositions.map { it.y }.toDoubleArray())
        graph.seriesMap.values.forEach { it.marker = None() }
        graph.styler.theme = MatlabTheme()
        GraphUtil.saveGraph("sim/tankPIDVA", graph)
    }

    @Test
    fun simulateRamseteFollower() {
        val dt = 1.0 / SIMULATION_HZ

        val trajectory = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), constraints = CONSTRAINTS)
                .splineTo(Vector2d(15.0, 15.0), PI)
                .splineTo(Vector2d(5.0, 35.0), PI / 3)
                .build()

        val clock = SimulatedClock()
        val drive = SimulatedTankDrive(dt, kV, TRACK_WIDTH)
        val follower = RamseteFollower(
                1.6,
                0.9,
            Pose2d(0.5, 0.5, Math.toRadians(3.0)),
                1.0,
                clock
        )
        follower.followTrajectory(trajectory)

        val targetPositions = mutableListOf<Vector2d>()
        val actualPositions = mutableListOf<Vector2d>()

        drive.poseEstimate = trajectory.start()
        var t = 0.0
        while (follower.isFollowing()) {
            t += dt
            clock.time = t
            val signal = follower.update(drive.poseEstimate)
            drive.setDriveSignal(signal)
            drive.updatePoseEstimate()

            targetPositions.add(trajectory[t].vec())
            actualPositions.add(drive.poseEstimate.vec())
        }

        val graph = XYChart(600, 400)
        graph.title = "Tank Ramsete Follower Sim (%.2fs)".format(t)
        graph.addSeries(
                "Target Trajectory",
                targetPositions.map { it.x }.toDoubleArray(),
                targetPositions.map { it.y }.toDoubleArray())
        graph.addSeries(
                "Actual Trajectory",
                actualPositions.map { it.x }.toDoubleArray(),
                actualPositions.map { it.y }.toDoubleArray())
        graph.seriesMap.values.forEach { it.marker = None() }
        graph.styler.theme = MatlabTheme()
        GraphUtil.saveGraph("sim/tankRamsete", graph)
    }

    @Test
    fun simulateGVFFollower() {
        val dt = 1.0 / SIMULATION_HZ

        val path = PathBuilder(Pose2d(0.0, 0.0, 0.0))
                .splineTo(Vector2d(15.0, 15.0), 0.0)
                .lineTo(Vector2d(30.0, 15.0))
                .build()

        val clock = SimulatedClock()
        val drive = SimulatedTankDrive(dt, kV, TRACK_WIDTH)
        val follower = TankGVFFollower(
                CONSTRAINTS,
                Pose2d(0.5, 0.5, Math.toRadians(3.0)),
                1.0,
                2.0,
                clock = clock)
        follower.followPath(path)

        val actualPositions = mutableListOf<Vector2d>()

        drive.poseEstimate = Pose2d(0.0, 10.0, -PI / 2)
        var t = 0.0
        while (follower.isFollowing()) {
            t += dt
            clock.time = t
            val signal = follower.update(drive.poseEstimate)
            drive.setDriveSignal(signal)
            drive.updatePoseEstimate()

            actualPositions.add(drive.poseEstimate.vec())
        }

        val pathPoints = DoubleProgression.fromClosedInterval(0.0, path.length(), 10_000)
                .map { path[it] }
        val graph = XYChart(600, 400)
        graph.title = "Tank GVF Follower Sim (%.2fs)".format(t)
        graph.addSeries(
                "Target Path",
                pathPoints.map { it.x }.toDoubleArray(),
                pathPoints.map { it.y }.toDoubleArray())
        graph.addSeries(
                "Actual Path",
                actualPositions.map { it.x }.toDoubleArray(),
                actualPositions.map { it.y }.toDoubleArray())
        graph.seriesMap.values.forEach { it.marker = None() }
        graph.styler.theme = MatlabTheme()
        GraphUtil.saveGraph("sim/tankGVF", graph)
    }
}
