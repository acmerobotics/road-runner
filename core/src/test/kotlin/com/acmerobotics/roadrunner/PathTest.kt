package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.TestUtil.assertDerivEquals
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.path.PathSegment
import com.acmerobotics.roadrunner.path.QuinticSpline
import com.acmerobotics.roadrunner.util.DoubleProgression
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class PathTest {
    @Test
    fun testPathDerivatives() {
        val splineSegment = Path(PathSegment(QuinticSpline(
                QuinticSpline.Knot(0.0, 0.0, 20.0, 40.0),
                QuinticSpline.Knot(45.0, 35.0, 60.0, 10.0)
        )))

        val resolution = 1000
        val s = DoubleProgression.fromClosedInterval(0.0, splineSegment.length(), resolution)
        val ds = s.step

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
    fun testProgressionParam() {
        val path = Path(listOf(
            PathSegment(QuinticSpline(
                    QuinticSpline.Knot(0.0, 0.0, 20.0, 40.0),
                    QuinticSpline.Knot(45.0, 35.0, 60.0, 10.0)
            )), PathSegment(QuinticSpline(
                QuinticSpline.Knot(45.0, 35.0, 60.0, 10.0),
                QuinticSpline.Knot(55.0, 70.0, -20.0, 30.0)
        ))
        ))

        val dispProg = DoubleProgression.fromClosedInterval(0.0, path.length(), 20)
        val indiv = dispProg.map(path::reparam)
        val seq = path.reparam(dispProg).toList()
        indiv.zip(seq).forEach { (a, b) ->
            assertEquals(a, b, 1e-6)
        }
    }
}
