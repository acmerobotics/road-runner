package com.acmerobotics.splinelib

import com.acmerobotics.splinelib.path.LineSegment
import com.acmerobotics.splinelib.path.Path
import com.acmerobotics.splinelib.path.QuinticSplineSegment
import com.acmerobotics.splinelib.path.SplineInterpolator
import com.acmerobotics.splinelib.trajectory.DriveConstraints
import com.acmerobotics.splinelib.trajectory.PathTrajectorySegment
import com.acmerobotics.splinelib.trajectory.Trajectory
import jdk.nashorn.internal.ir.annotations.Ignore
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance


@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class PerformanceTest {
    companion object {
        private val CONSTRAINTS = DriveConstraints(50.0, 25.0, Math.PI / 2, Math.PI / 2, 500.0)
    }

    @Ignore
    @Test
    fun testTrajectoryGenerationPerformance() {
        while (true) {
            val startTime = System.currentTimeMillis()
            val line = LineSegment(
                    Vector2d(0.0, 0.0),
                    Vector2d(15.0, 15.0)
            )
            val spline = QuinticSplineSegment(
                    Waypoint(15.0, 15.0, 15.0, 15.0),
                    Waypoint(30.0, 15.0, 20.0, 5.0)
            )
            val trajectory = Trajectory(listOf(
                    PathTrajectorySegment(listOf(Path(line, SplineInterpolator(0.0, Math.PI / 4)), Path(spline)), listOf(CONSTRAINTS, CONSTRAINTS))
            ))
            val elapsedTime = System.currentTimeMillis() - startTime
            println("${elapsedTime}ms")
        }
    }
}