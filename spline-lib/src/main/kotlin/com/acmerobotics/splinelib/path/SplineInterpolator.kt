package com.acmerobotics.splinelib.path

class SplineInterpolator(private val startHeading: Double, private val endHeading: Double) : HeadingInterpolator {
    private val tangentInterpolator = TangentInterpolator()
    private lateinit var headingSpline: QuinticPolynomial
    private lateinit var parametricCurve: ParametricCurve

    override fun init(parametricCurve: ParametricCurve) {
        this.parametricCurve = parametricCurve

        tangentInterpolator.init(this.parametricCurve)

        headingSpline = QuinticPolynomial(
            startHeading,
            this.parametricCurve.internalTangentAngleDeriv(0.0),
            this.parametricCurve.internalTangentAngleSecondDeriv(0.0),
            endHeading,
            this.parametricCurve.internalTangentAngleDeriv(1.0),
            this.parametricCurve.internalTangentAngleSecondDeriv(1.0)
        )
    }

    override operator fun get(displacement: Double): Double {
        val t = parametricCurve.displacementToParameter(displacement)
        return headingSpline[t]
    }

    override fun deriv(displacement: Double): Double {
        val t = parametricCurve.displacementToParameter(displacement)
        return headingSpline.deriv(t) * parametricCurve.parameterDeriv(t)
    }

    override fun secondDeriv(displacement: Double): Double {
        val t = parametricCurve.displacementToParameter(displacement)
        return headingSpline.secondDeriv(t) * parametricCurve.parameterDeriv(t) * parametricCurve.parameterDeriv(t) +
                headingSpline.deriv(t) * parametricCurve.parameterSecondDeriv(t)
    }

}