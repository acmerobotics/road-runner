package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.PathContinuityViolationException
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import org.knowm.xchart.QuickChart
import org.knowm.xchart.style.MatlabTheme
import kotlin.math.max

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class TrajectorySpliceTest {

    private fun saveSplicedTrajectory(
        name: String,
        traj1: Trajectory,
        traj2: Trajectory,
        t: Double,
        resolution: Int = 1000
    ) {
        val duration = max(traj1.duration(), t + traj2.duration())
        val timeData = (0..resolution).map { it / resolution.toDouble() * duration }.toDoubleArray()

        val velData1 = timeData.map { traj1.velocity(it) }
        val xVelData1 = velData1.map { it.x }.toDoubleArray()
        val yVelData1 = velData1.map { it.y }.toDoubleArray()
        val omegaData1 = velData1.map { it.heading }.toDoubleArray()

        val velData2 = timeData.map { traj2.velocity(it - t) }
        val xVelData2 = velData2.map { it.x }.toDoubleArray()
        val yVelData2 = velData2.map { it.y }.toDoubleArray()
        val omegaData2 = velData2.map { it.heading }.toDoubleArray()

        val labels = listOf("dx1/dt", "dy1/dt", "ω1", "dx2/dt", "dy2/dt", "ω2")
        val data = listOf(xVelData1, yVelData1, omegaData1, xVelData2, yVelData2, omegaData2)

        val graph = QuickChart.getChart(
            name,
            "time (sec)",
            "",
            labels.toTypedArray(),
            timeData,
            data.toTypedArray()
        )
        graph.styler.theme = MatlabTheme()

        GraphUtil.saveGraph(name, graph)
    }

    @Test
    fun testSpliceTangentHeading() {
        val constraints = DriveConstraints(25.0, 50.0, 50.0, 1.0, 1.0, 1.0)

        val traj1 = TrajectoryBuilder(Pose2d(), constraints = constraints)
            .splineTo(Pose2d(40.0, 50.0))
            .build()

        val t = 0.6 * traj1.duration()

        val traj2 = TrajectoryBuilder(traj1, t, constraints)
            .splineTo(Pose2d(50.0, 60.0))
            .build()

        saveSplicedTrajectory("splice/tangentHeading", traj1, traj2, t)
    }

    @Test
    fun testSpliceSplineHeading() {
        val constraints = DriveConstraints(25.0, 50.0, 50.0, 1.0, 1.0, 1.0)

        val traj1 = TrajectoryBuilder(Pose2d(), constraints = constraints)
            .splineTo(Pose2d(40.0, 50.0))
            .build()

        val t = 0.6 * traj1.duration()

        val traj2 = TrajectoryBuilder(traj1, t, constraints)
            .splineToSplineHeading(Pose2d(50.0, 60.0), Math.toRadians(15.0))
            .build()

        saveSplicedTrajectory("splice/splineHeading", traj1, traj2, t)
    }

    @Test
    fun testSpliceTangentHeadingException() {
        val constraints = DriveConstraints(25.0, 50.0, 50.0, 1.0, 1.0, 1.0)

        val traj1 = TrajectoryBuilder(Pose2d(), constraints = constraints)
            .splineToConstantHeading(Pose2d(40.0, 50.0))
            .build()

        val t = 0.6 * traj1.duration()

        try {
            TrajectoryBuilder(traj1, t, constraints)
                .splineTo(Pose2d(50.0, 60.0))
                .build()
            assert(false)
        } catch (e: PathContinuityViolationException) {
            assert(true)
        }
    }
}
