package com.acmerobotics.splinelib

import com.acmerobotics.splinelib.TestUtil.compareDerivatives
import com.acmerobotics.splinelib.path.*
import com.acmerobotics.splinelib.trajectory.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class WheelProfileTest {
    companion object {
        private val CONSTRAINTS = DriveConstraints(50.0, 25.0, Math.PI / 2, Math.PI / 2, 500.0, TankModifier(12.0))
    }

    @Test
    fun testWheelProfileDerivatives() {
        val line = LineSegment(
                Vector2d(0.0, 0.0),
                Vector2d(15.0, 15.0)
        )
        val spline = QuinticSplineSegment(
                Waypoint(15.0, 15.0, 15.0, 15.0),
                Waypoint(30.0, 15.0, 20.0, 5.0)
        )
        val trajectory = Trajectory(listOf(
                PathTrajectorySegment(listOf(Path(line), Path(spline)), listOf(CONSTRAINTS, CONSTRAINTS))
        ))
        val wheelProfiles = trajectory.modify(TankModifier(12.0))

        val resolution = 1000
        val dt = trajectory.duration() / resolution.toDouble()
        val t = (0..resolution).map { it * dt }

        for (wheelProfile in wheelProfiles) {
            val x = t.map { wheelProfile[it].x }
            val v = t.map { wheelProfile[it].v }
            val a = t.map { wheelProfile[it].a }

            assert(compareDerivatives(x, v, dt, 2.0))
            assert(compareDerivatives(v, a, dt, 2.0))
        }
    }
}