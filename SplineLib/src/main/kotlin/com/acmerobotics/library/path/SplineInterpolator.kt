package com.acmerobotics.library.path

class SplineInterpolator(private val startHeading: Double, private val endHeading: Double) : HeadingInterpolator {
    private val tangentInterpolator = TangentInterpolator()
    private lateinit var headingSpline: QuinticPolynomial
    private lateinit var splineSegment: QuinticSplineSegment

    override fun init(parametricCurve: ParametricCurve) {
        this.splineSegment = splineSegment

        tangentInterpolator.init(splineSegment)

        headingSpline = QuinticPolynomial(
            startHeading,
            splineSegment.internalTangentAngleDeriv(0.0),
            splineSegment.internalTangentAngleSecondDeriv(0.0),
            endHeading,
            splineSegment.internalTangentAngleDeriv(1.0),
            splineSegment.internalTangentAngleSecondDeriv(1.0)
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