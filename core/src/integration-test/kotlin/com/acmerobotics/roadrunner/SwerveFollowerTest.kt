package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.SwerveDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.SwerveConstraints
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
private val CONSTRAINTS = SwerveConstraints(BASE_CONSTRAINTS, TRACK_WIDTH)

private val VOLTAGE_NOISE_DIST = NormalDistribution(1.0, 0.05)

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class SwerveFollowerTest {

    private class SimulatedSwerveDrive(
        private val dt: Double,
        private val kV: Double,
        trackWidth: Double,
        wheelBase: Double = trackWidth
    ) : SwerveDrive(kV, 0.0, 0.0, trackWidth, wheelBase) {
        override val rawExternalHeading = Double.NaN
        var powers = listOf(0.0, 0.0, 0.0, 0.0)
        var positions = listOf(0.0, 0.0, 0.0, 0.0)
        var orientations = listOf(0.0, 0.0, 0.0, 0.0)

        init {
            localizer = SwerveLocalizer(this, false)
        }

        override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
            powers = listOf(frontLeft, rearLeft, rearRight, frontRight)
                    .map { it * VOLTAGE_NOISE_DIST.sample() }
                    .map { max(-1.0, min(it, 1.0)) }
            positions = positions.zip(powers)
                    .map { it.first + it.second / kV * dt }
        }

        override fun getWheelPositions() = positions

        override fun setModuleOrientations(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
            orientations = listOf(frontLeft, rearLeft, rearRight, frontRight)
        }

        override fun getModuleOrientations() = orientations
    }

    @Test
    fun simulatePIDVAFollower() {
        val dt = 1.0 / SIMULATION_HZ

        val trajectory = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), constraints = CONSTRAINTS)
                .splineTo(Vector2d(15.0, 15.0), PI)
                .splineTo(Vector2d(5.0, 35.0), PI / 3)
                .build()

        val clock = SimulatedClock()
        val drive = SimulatedSwerveDrive(dt, kV, TRACK_WIDTH, TRACK_WIDTH)
        val follower = HolonomicPIDVAFollower(
            PIDCoefficients(0.1),
            PIDCoefficients(0.1),
            PIDCoefficients(0.1),
            Pose2d(0.5, 0.5, Math.toRadians(3.0)),
            1.0,
            clock
        )
        follower.followTrajectory(trajectory)

        val targetPositions = mutableListOf<Vector2d>()
        val actualPositions = mutableListOf<Vector2d>()

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
        graph.title = "Swerve PIDVA Follower Sim (%.2fs)".format(t)
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
        GraphUtil.saveGraph("sim/swervePIDVA", graph)
    }
}
