package com.acmerobotics.splinelib

import com.acmerobotics.splinelib.MecanumFollowerTest.Companion.SIMULATION_HZ
import com.acmerobotics.splinelib.MecanumFollowerTest.Companion.TRACK_WIDTH
import com.acmerobotics.splinelib.MecanumFollowerTest.Companion.kV
import com.acmerobotics.splinelib.control.PIDCoefficients
import com.acmerobotics.splinelib.drive.MecanumDrive
import com.acmerobotics.splinelib.drive.MecanumKinematics
import com.acmerobotics.splinelib.followers.MecanumPIDVAFollower
import com.acmerobotics.splinelib.trajectory.DriveConstraints
import com.acmerobotics.splinelib.trajectory.MecanumConstraints
import com.acmerobotics.splinelib.trajectory.TrajectoryBuilder
import org.apache.commons.math3.distribution.NormalDistribution
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.markers.None
import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.min

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class MecanumFollowerTest {
    companion object {
        const val kV = 1.0 / 60.0
        const val SIMULATION_HZ = 25
        const val TRACK_WIDTH = 3.0

        private val BASE_CONSTRAINTS = DriveConstraints(50.0, 25.0, Math.PI / 2, Math.PI / 2)
        private val CONSTRAINTS = MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH)
    }

    // TODO: fix simulation
    private class SimulatedMecanumDrive(
            private val dt: Double,
            private val kV: Double,
            trackWidth: Double,
            wheelBase: Double = trackWidth
    ) : MecanumDrive(trackWidth, wheelBase) {
        companion object {
//            val VOLTAGE_NOISE_DIST = NormalDistribution(0.0, 0.25 / 12.0)
            val VOLTAGE_NOISE_DIST = NormalDistribution(1.0, 0.05)

            fun clamp(value: Double, min: Double, max: Double) = min(max, max(min, value))
        }

        var powers = listOf(0.0, 0.0, 0.0, 0.0)
        var positions = listOf(0.0, 0.0, 0.0, 0.0)

        override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
            powers = listOf(frontLeft, rearLeft, rearRight, frontRight)
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

        val drive = SimulatedMecanumDrive(dt, kV, TRACK_WIDTH)
        val follower = MecanumPIDVAFollower(drive, PIDCoefficients(1.0), PIDCoefficients(5.0), kV, 0.0, 0.0)
        follower.followTrajectory(trajectory, 0.0)

        val targetPositions = mutableListOf<Vector2d>()
        val actualPositions = mutableListOf<Vector2d>()

        val samples = ceil(trajectory.duration() / dt).toInt()
        for (sample in 0..samples) {
            val t = sample * dt
            follower.update(drive.getPoseEstimate(), t)
            drive.updatePoseEstimate(t)

            targetPositions.add(trajectory[t].pos())
            actualPositions.add(drive.getPoseEstimate().pos())
        }

        val graph = XYChart(600, 400)
        graph.title = "Mecanum PIDVA Follower Sim"
        graph.addSeries(
                "Target Trajectory",
                targetPositions.map { it.x }.toDoubleArray(),
                targetPositions.map { it.y }.toDoubleArray())
        graph.addSeries(
                "Actual Trajectory",
                actualPositions.map { it.x }.toDoubleArray(),
                actualPositions.map { it.y }.toDoubleArray())
        graph.seriesMap.values.forEach { it.marker = None() }
        GraphUtil.saveGraph("mecanumSim", graph)
    }
}
