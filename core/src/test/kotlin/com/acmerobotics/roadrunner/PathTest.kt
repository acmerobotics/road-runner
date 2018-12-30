package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.TestUtil.assertDerivEquals
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.path.QuinticSplineSegment
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class PathTest {
    @Test
    fun testPathDerivatives() {
        val splineSegment = Path(QuinticSplineSegment(
                QuinticSplineSegment.Waypoint(0.0, 0.0, 20.0, 40.0),
                QuinticSplineSegment.Waypoint(45.0, 35.0, 60.0, 10.0)
        ))

        val resolution = 1000
        val ds = splineSegment.length() / resolution.toDouble()
        val s = (0..resolution).map { it * ds }

        val x = s.map { splineSegment[it].x }
        val dx = s.map { splineSegment.deriv(it).x }
        val d2x = s.map { splineSegment.secondDeriv(it).x }

        val y = s.map { splineSegment[it].y }
        val dy = s.map { splineSegment.deriv(it).y }
        val d2y = s.map { splineSegment.secondDeriv(it).y }

        val heading = s.map { splineSegment[it].heading }
        val headingDeriv = s.map { splineSegment.deriv(it).heading }
        val headingSecondDeriv = s.map { splineSegment.secondDeriv(it).heading }
        
        assertDerivEquals(x, dx, ds, 0.01)
        assertDerivEquals(dx, d2x, ds, 0.01)

        assertDerivEquals(y, dy, ds, 0.01)
        assertDerivEquals(dy, d2y, ds, 0.01)

        assertDerivEquals(heading, headingDeriv, ds, 0.01)
        assertDerivEquals(headingDeriv, headingSecondDeriv, ds, 0.01)
    }

    @Test
    fun test() {
        println(TrajectoryBuilder(Pose2d(10.0, 20.0, 0.0), DriveConstraints(10.0, 10.0, Math.PI / 2, Math.PI / 2))
                .splineTo(Pose2d(0.0, 23.0, -Math.PI / 2))
                .build()[0.0])
    }
}