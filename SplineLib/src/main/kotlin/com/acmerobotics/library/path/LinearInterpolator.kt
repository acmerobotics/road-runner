package com.acmerobotics.library.path

class LinearInterpolator(private var startAngle: Double = Double.NaN, private var endAngle: Double = Double.NaN) : HeadingInterpolator() {
    private var rotAngle: Double = 0.0

    override fun init(path: Path) {
        super.init(path)

        if (startAngle.isNaN()) {
            val startDeriv = path.deriv(0.0)
            startAngle = Math.atan2(startDeriv.y, startDeriv.x)
        }

        if (endAngle.isNaN()) {
            val endDeriv = path.deriv(path.length())
            endAngle = Math.atan2(endDeriv.y, endDeriv.x)
        }

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