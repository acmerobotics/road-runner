package com.acmerobotics.library.path.heading

import com.acmerobotics.library.path.parametric.QuinticPolynomial
import com.acmerobotics.library.path.parametric.QuinticSplineSegment

class SplineInterpolator(private val startHeading: Double, private val endHeading: Double) : HeadingInterpolator {
    private val tangentInterpolator = TangentInterpolator()
    private lateinit var headingSpline: QuinticPolynomial
    private lateinit var splineSegment: QuinticSplineSegment

    override fun fit(splineSegment: QuinticSplineSegment) {
        this.splineSegment = splineSegment

        tangentInterpolator.fit(splineSegment)

        val tangentStartDeriv = tangentInterpolator.deriv(0.0)
        val tangentEndDeriv = tangentInterpolator.deriv(1.0)
        val tangentStartSecondDeriv = tangentInterpolator.secondDeriv(0.0)
        val tangentEndSecondDeriv = tangentInterpolator.secondDeriv(1.0)

        val parameterStartDeriv = splineSegment.parameterDeriv(0.0)
        val parameterEndDeriv = splineSegment.parameterDeriv(1.0)
        val parameterStartSecondDeriv = splineSegment.parameterSecondDeriv(0.0)
        val parameterEndSecondDeriv = splineSegment.parameterSecondDeriv(1.0)

        headingSpline = QuinticPolynomial(
            startHeading,
            tangentStartDeriv / parameterStartDeriv,
            (tangentStartSecondDeriv - tangentStartDeriv / parameterStartDeriv * parameterStartSecondDeriv) /
                    (parameterStartDeriv * parameterStartDeriv),
            endHeading,
            tangentEndDeriv / parameterEndDeriv,
            (tangentEndSecondDeriv - tangentEndDeriv / parameterEndDeriv * parameterEndSecondDeriv) /
                    (parameterEndDeriv * parameterEndDeriv)
        )
    }

    override operator fun get(displacement: Double): Double {
        val t = splineSegment.displacementToParameter(displacement)
        return headingSpline[t]
    }

    override fun deriv(displacement: Double): Double {
        val t = splineSegment.displacementToParameter(displacement)
        return headingSpline.deriv(t) * splineSegment.parameterDeriv(t)
    }

    override fun secondDeriv(displacement: Double): Double {
        val t = splineSegment.displacementToParameter(displacement)
        return headingSpline.secondDeriv(t) * splineSegment.parameterDeriv(t) * splineSegment.parameterDeriv(t) +
                headingSpline.deriv(t) * splineSegment.parameterSecondDeriv(t)
    }

}