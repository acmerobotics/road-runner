package com.acmerobotics.library

class LinearInterpolator(private val path: Path) : HeadingInterpolator {
    private val startAngle: Double
    private val rotAngle: Double

    init {
        val startDeriv = path.deriv(0.0)
        val endDeriv = path.deriv(path.length())

        startAngle = Math.atan2(startDeriv.y, startDeriv.x)
        val endAngle = Math.atan2(endDeriv.y, endDeriv.x)

        rotAngle = if (endAngle >= startAngle) {
            endAngle - startAngle
        } else {
            2 * Math.PI - endAngle + startAngle
        }
    }

    override fun get(displacement: Double) = (startAngle + displacement / path.length() * rotAngle) % (2 * Math.PI)

    override fun deriv(displacement: Double) = rotAngle / path.length()

    override fun secondDeriv(displacement: Double) = 0.0
}