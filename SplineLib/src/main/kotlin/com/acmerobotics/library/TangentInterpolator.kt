package com.acmerobotics.library

class TangentInterpolator : HeadingInterpolator {
    private lateinit var path: Path

    override fun init(path: Path) {
        this.path = path
    }

    override fun get(displacement: Double): Double {
        val pathDeriv = path.deriv(displacement)
        return Math.atan2(pathDeriv.y, pathDeriv.x)
    }

    override fun deriv(displacement: Double): Double {
        val pathDeriv = path.deriv(displacement)
        val pathSecondDeriv = path.secondDeriv(displacement)

        val dydx = pathDeriv.y / pathDeriv.x
        val d2ydx2 = pathSecondDeriv.y / pathSecondDeriv.x

        return d2ydx2 / (1 + dydx * dydx)
    }

    override fun secondDeriv(displacement: Double): Double {
        val pathDeriv = path.deriv(displacement)
        val pathSecondDeriv = path.secondDeriv(displacement)
        val pathThirdDeriv = path.thirdDeriv(displacement)

        val dydx = pathDeriv.y / pathDeriv.x
        val d2ydx2 = pathSecondDeriv.y / pathSecondDeriv.x
        val d3ydx3 = pathThirdDeriv.y / pathThirdDeriv.x

        var alpha = (1 + dydx * dydx) * d3ydx3 - d2ydx2 * 2 * dydx * d2ydx2
        alpha /= (1 + dydx * dydx) * (1 + dydx * dydx)

        return alpha
    }
}