package com.acmerobotics.splinelib

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
        const val TRACK_WIDTH = 1.0

        private val BASE_CONSTRAINTS = DriveConstraints(50.0, 25.0, Math.PI / 2, Math.PI / 2)
        private val CONSTRAINTS = MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH)
    }

    private class SimulatedMecanumDrive(trackWidth: Double, wheelBase: Double = trackWidth) : MecanumDrive(trackWidth, wheelBase) {
        companion object {
//            val VOLTAGE_NOISE_DIST = NormalDistribution(0.0, 0.25 / 12.0)
            val VOLTAGE_NOISE_DIST = NormalDistribution(1.0, 0.05)

            fun clamp(value: Double, min: Double, max: Double) = min(max, max(min, value))
        }

        var frontLeftPower: Double = 0.0
        var rearLeftPower: Double = 0.0
        var rearRightPower: Double = 0.0
        var frontRightPower: Double = 0.0

        override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
            frontLeftPower = clamp(frontLeft * VOLTAGE_NOISE_DIST.sample(), 0.0, 1.0)
            rearLeftPower = clamp(rearLeft * VOLTAGE_NOISE_DIST.sample(), 0.0, 1.0)
            rearRightPower = clamp(rearRight * VOLTAGE_NOISE_DIST.sample(), 0.0, 1.0)
            frontRightPower = clamp(frontRight * VOLTAGE_NOISE_DIST.sample(), 0.0, 1.0)
        }
    }

    @Test
    fun simulatePIDVAFollower() {
        val dt = 1.0 / SIMULATION_HZ

        val trajectory = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), CONSTRAINTS)
                .splineTo(Pose2d(15.0, 15.0, Math.PI))
                .splineTo(Pose2d(5.0, 35.0, Math.PI / 3))
                .waitFor(0.5)
                .build()

        val drive = SimulatedMecanumDrive(TRACK_WIDTH)
        val follower = MecanumPIDVAFollower(drive, PIDCoefficients(1.0), PIDCoefficients(5.0), kV, 0.0, 0.0)
        follower.followTrajectory(trajectory, 0.0)

        val targetPositions = mutableListOf<Vector2d>()
        val actualPositions = mutableListOf<Vector2d>()

        var pose = trajectory.start()
        val samples = ceil(trajectory.duration() / dt).toInt()
        for (sample in 0..samples) {
            val t = sample * dt
            follower.update(pose, t)
            val wheelVelocities = listOf(
                    drive.frontLeftPower / kV,
                    drive.rearLeftPower / kV,
                    drive.rearRightPower / kV,
                    drive.frontRightPower / kV
            )
            val robotPoseVelocity = MecanumKinematics.wheelToRobotVelocities(wheelVelocities, drive.trackWidth)
            val robotPoseUpdate = robotPoseVelocity * dt
            val poseUpdate = Pose2d(robotPoseUpdate.pos().rotated(pose.heading), robotPoseUpdate.heading)
            pose += poseUpdate

            targetPositions.add(trajectory[t].pos())
            actualPositions.add(pose.pos())
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
