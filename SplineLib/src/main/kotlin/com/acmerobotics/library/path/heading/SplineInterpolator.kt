package com.acmerobotics.library.path.heading

import com.acmerobotics.library.path.parametric.ParametricCurve

class SplineInterpolator(private var startAngle: Double = Double.NaN, private var endAngle: Double = Double.NaN) : HeadingInterpolator() {
    private var rotAngle: Double = 0.0

    override fun init(parametricCurve: ParametricCurve) {
        super.init(parametricCurve)

        if (startAngle.isNaN()) {
            val startDeriv = parametricCurve.deriv(0.0)
            startAngle = Math.atan2(startDeriv.y, startDeriv.x)
        }

        if (endAngle.isNaN()) {
            val endDeriv = parametricCurve.deriv(parametricCurve.length())
            endAngle = Math.atan2(endDeriv.y, endDeriv.x)
        }

        rotAngle = if (endAngle >= startAngle) {
            endAngle - startAngle
        } else {
            2 * Math.PI - endAngle + startAngle
        }
    }

    override fun get(displacement: Double) = (startAngle + displacement / parametricCurve.length() * rotAngle) % (2 * Math.PI)

    override fun deriv(displacement: Double) = rotAngle / parametricCurve.length()

    override fun secondDeriv(displacement: Double) = 0.0
}