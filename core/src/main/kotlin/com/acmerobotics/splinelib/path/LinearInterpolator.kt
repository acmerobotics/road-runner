package com.acmerobotics.splinelib.path

class LinearInterpolator(private val startHeading: Double, endHeading: Double) : HeadingInterpolator {
    private val turnAngle: Double = if (endHeading >= startHeading) {
        endHeading - startHeading
    } else {
        2 * Math.PI - endHeading + startHeading
    }
    private var length: Double = 0.0

    override fun init(parametricCurve: ParametricCurve) {
        length = parametricCurve.length()
    }
    
    override fun get(displacement: Double) = (startHeading + displacement / length * turnAngle) % (2 * Math.PI)

    override fun deriv(displacement: Double) = turnAngle / length

    override fun secondDeriv(displacement: Double) = 0.0
}