package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.PathContinuityViolationException
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint
import com.acmerobotics.roadrunner.util.DoubleProgression
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import org.knowm.xchart.QuickChart
import org.knowm.xchart.style.MatlabTheme
import kotlin.math.PI
import kotlin.math.max

private val VEL_CONSTRAINT = MinVelocityConstraint(
    listOf(
        TranslationalVelocityConstraint(50.0),
        AngularVelocityConstraint(PI / 2)
    )
)
private val ACCEL_CONSTRAINT = ProfileAccelerationConstraint(10.0)

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
        val timeData = DoubleProgression.fromClosedInterval(0.0, duration, resolution).toList().toDoubleArray()

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
        val traj1 = TrajectoryBuilder(
            Pose2d(),
            baseVelConstraint = VEL_CONSTRAINT,
            baseAccelConstraint = ACCEL_CONSTRAINT
        )
            .splineTo(Vector2d(40.0, 50.0), 0.0)
            .build()

        val t = 0.6 * traj1.duration()

        val traj2 = TrajectoryBuilder(traj1, t, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
            .splineTo(Vector2d(50.0, 60.0), 0.0)
            .build()

        saveSplicedTrajectory("splice/tangentHeading", traj1, traj2, t)
    }

    @Test
    fun testSpliceSplineHeading() {
        val traj1 = TrajectoryBuilder(
            Pose2d(),
            baseVelConstraint = VEL_CONSTRAINT,
            baseAccelConstraint = ACCEL_CONSTRAINT
        )
            .splineTo(Vector2d(40.0, 50.0), 0.0)
            .build()

        val t = 0.6 * traj1.duration()

        val traj2 = TrajectoryBuilder(traj1, t, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
            .splineToSplineHeading(Pose2d(50.0, 60.0), Math.toRadians(15.0))
            .build()

        saveSplicedTrajectory("splice/splineHeading", traj1, traj2, t)
    }

    @Test
    fun testSpliceTangentHeadingException() {
        val traj1 = TrajectoryBuilder(
            Pose2d(),
            baseVelConstraint = VEL_CONSTRAINT,
            baseAccelConstraint = ACCEL_CONSTRAINT
        )
            .splineToConstantHeading(Vector2d(40.0, 50.0), 0.0)
            .build()

        val t = 0.6 * traj1.duration()

        try {
            TrajectoryBuilder(traj1, t, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
                .splineTo(Vector2d(50.0, 60.0), 0.0)
                .build()
            assert(false)
        } catch (e: PathContinuityViolationException) {
            assert(true)
        }
    }
}
