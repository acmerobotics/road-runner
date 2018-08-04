package com.acmerobotics.splinelib

import com.acmerobotics.splinelib.control.PIDCoefficients
import com.acmerobotics.splinelib.drive.TankDrive
import com.acmerobotics.splinelib.followers.GVFFollower
import com.acmerobotics.splinelib.followers.RamseteFollower
import com.acmerobotics.splinelib.followers.TankPIDVAFollower
import com.acmerobotics.splinelib.path.Path
import com.acmerobotics.splinelib.path.QuinticSplineSegment
import com.acmerobotics.splinelib.profile.SimpleMotionConstraints
import com.acmerobotics.splinelib.trajectory.DriveConstraints
import com.acmerobotics.splinelib.trajectory.TankConstraints
import com.acmerobotics.splinelib.trajectory.TrajectoryBuilder
import org.apache.commons.math3.distribution.NormalDistribution
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.markers.None
import kotlin.math.atan
import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.min


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class TankFollowerTest {
    companion object {
        const val kV = 1.0 / 60.0
        const val SIMULATION_HZ = 25
        const val TRACK_WIDTH = 3.0

        private val BASE_CONSTRAINTS = DriveConstraints(50.0, 25.0, Math.PI / 2, Math.PI / 2)
        private val CONSTRAINTS = TankConstraints(BASE_CONSTRAINTS, TRACK_WIDTH)
    }

    private class SimulatedTankDrive(
            private val dt: Double,
            private val kV: Double,
            trackWidth: Double
    ) : TankDrive(trackWidth) {
        companion object {
//            val VOLTAGE_NOISE_DIST = NormalDistribution(0.0, 0.25 / 12.0)
            val VOLTAGE_NOISE_DIST = NormalDistribution(1.0, 0.05)

            fun clamp(value: Double, min: Double, max: Double) = min(max, max(min, value))
        }

        var powers = listOf(0.0, 0.0)
        var positions = listOf(0.0, 0.0)

        override fun setMotorPowers(left: Double, right: Double) {
            powers = listOf(left, right)
                    .map { it * VOLTAGE_NOISE_DIST.sample() }
                    .map { clamp(it, 0.0, 1.0) }
        }

        override fun getMotorPositions(): List<Double> = positions

        override fun updatePoseEstimate(timestamp: Double) {
            positions = positions.zip(powers)
                    .map { it.first + it.second / kV * dt }
            super.updatePoseEstimate(timestamp)
        }
    }

    @Test
    fun simulatePIDVAFollower() {
        val dt = 1.0 / SIMULATION_HZ

        val trajectory = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), CONSTRAINTS)
                .beginComposite()
                .splineTo(Pose2d(15.0, 15.0, Math.PI))
                .splineTo(Pose2d(5.0, 35.0, Math.PI / 3))
                .closeComposite()
                .waitFor(0.5)
                .build()

        val drive = SimulatedTankDrive(dt, kV, TRACK_WIDTH)
        val follower = TankPIDVAFollower(drive, PIDCoefficients(1.0), PIDCoefficients(kP = 1.0), kV, 0.0, 0.0)
        follower.followTrajectory(trajectory, 0.0)

        val targetPositions = mutableListOf<Vector2d>()
        val actualPositions = mutableListOf<Vector2d>()

        drive.resetPoseEstimate(trajectory.start())
        val samples = ceil(trajectory.duration() / dt).toInt()
        for (sample in 1..samples) {
            val t = sample * dt
            follower.update(drive.getPoseEstimate(), t)
            drive.updatePoseEstimate(t)

            targetPositions.add(trajectory[t].pos())
            actualPositions.add(drive.getPoseEstimate().pos())
        }

        val graph = XYChart(600, 400)
        graph.title = "Tank PIDVA Follower Sim"
        graph.addSeries(
                "Target Trajectory",
                targetPositions.map { it.x }.toDoubleArray(),
                targetPositions.map { it.y }.toDoubleArray())
        graph.addSeries(
                "Actual Trajectory",
                actualPositions.map { it.x }.toDoubleArray(),
                actualPositions.map { it.y }.toDoubleArray())
        graph.seriesMap.values.forEach { it.marker = None() }
        GraphUtil.saveGraph("tankPIDVASim", graph)
    }

    @Test
    fun simulateRamseteFollower() {
        val dt = 1.0 / SIMULATION_HZ

        val trajectory = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), CONSTRAINTS)
                .beginComposite()
                .splineTo(Pose2d(15.0, 15.0, Math.PI))
                .splineTo(Pose2d(5.0, 35.0, Math.PI / 3))
                .closeComposite()
                .waitFor(0.5)
                .build()

        val drive = SimulatedTankDrive(dt, kV, TRACK_WIDTH)
        val follower = RamseteFollower(drive, 0.0008, 0.5, kV, 0.0, 0.0)
        follower.followTrajectory(trajectory, 0.0)

        val targetPositions = mutableListOf<Vector2d>()
        val actualPositions = mutableListOf<Vector2d>()

        drive.resetPoseEstimate(trajectory.start())
        val samples = ceil(trajectory.duration() / dt).toInt()
        for (sample in 1..samples) {
            val t = sample * dt
            follower.update(drive.getPoseEstimate(), t)
            drive.updatePoseEstimate(t)

            targetPositions.add(trajectory[t].pos())
            actualPositions.add(drive.getPoseEstimate().pos())
        }

        val graph = XYChart(600, 400)
        graph.title = "Tank Ramsete Follower Sim"
        graph.addSeries(
                "Target Trajectory",
                targetPositions.map { it.x }.toDoubleArray(),
                targetPositions.map { it.y }.toDoubleArray())
        graph.addSeries(
                "Actual Trajectory",
                actualPositions.map { it.x }.toDoubleArray(),
                actualPositions.map { it.y }.toDoubleArray())
        graph.seriesMap.values.forEach { it.marker = None() }
        GraphUtil.saveGraph("tankRamseteSim", graph)
    }

    @Test
    fun simulateGVFFollower() {
        val dt = 1.0 / SIMULATION_HZ

        val path = Path(QuinticSplineSegment(
                Waypoint(0.0, 0.0, 20.0, 0.0),
                Waypoint(15.0, 15.0, 20.0, 0.0)
        ))

        val drive = SimulatedTankDrive(dt, kV, TRACK_WIDTH)
        val follower = GVFFollower(
                drive,
                SimpleMotionConstraints(5.0, 25.0),
                3.0,
                5.0,
                kV,
                0.0,
                0.0,
                ::atan)
        follower.followPath(path, 0.0)

        val actualPositions = mutableListOf<Vector2d>()

        drive.resetPoseEstimate(Pose2d(0.0, 10.0, -Math.PI / 2))
        var t = 0.0
        while (follower.isFollowing()) {
            t += dt
            follower.update(drive.getPoseEstimate(), t)
            drive.updatePoseEstimate(t)

            actualPositions.add(drive.getPoseEstimate().pos())
        }

        val pathPoints = (0..10000)
                .map { it / 10000.0 * path.length() }
                .map { path[it] }
        val graph = XYChart(600, 400)
        graph.title = "Tank GVF Follower Sim"
        graph.addSeries(
                "Target Trajectory",
                pathPoints.map { it.x }.toDoubleArray(),
                pathPoints.map { it.y }.toDoubleArray())
        graph.addSeries(
                "Actual Trajectory",
                actualPositions.map { it.x }.toDoubleArray(),
                actualPositions.map { it.y }.toDoubleArray())
        graph.seriesMap.values.forEach { it.marker = None() }
        GraphUtil.saveGraph("tankGVFSim", graph)
    }
}
