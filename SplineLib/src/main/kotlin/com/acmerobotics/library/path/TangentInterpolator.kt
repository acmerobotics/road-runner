package com.acmerobotics.library.path

class TangentInterpolator : HeadingInterpolator() {
    override fun get(displacement: Double): Double {
        val pathDeriv = path.deriv(displacement)
        val angle = Math.atan2(pathDeriv.y, pathDeriv.x)
        return if (angle.isNaN()) 0.0 else angle
    }

    override fun deriv(displacement: Double): Double {
        val pathDeriv = path.deriv(displacement)
        val pathSecondDeriv = path.secondDeriv(displacement)

        var deriv = pathDeriv.x * pathSecondDeriv.y - pathSecondDeriv.x * pathDeriv.y
        deriv /= (pathDeriv.x * pathDeriv.x + pathDeriv.y * pathDeriv.y)

        return if (deriv.isNaN()) 0.0 else deriv
    }

    override fun secondDeriv(displacement: Double): Double {
        val pathDeriv = path.deriv(displacement)
        val pathSecondDeriv = path.secondDeriv(displacement)
        val pathThirdDeriv = path.thirdDeriv(displacement)

        // http://www.wolframalpha.com/input/?i=d%2Fdt(d%2Fdt(arctan((dy%2Fdt)%2F(dx%2Fdt))))
        // I hate everything, especially chain rule
        val denominator = pathDeriv.x * pathDeriv.x + pathDeriv.y * pathDeriv.y
        val firstTerm = (pathThirdDeriv.y - pathThirdDeriv.x) / denominator
        var secondTerm = (pathDeriv.x * pathSecondDeriv.y - pathSecondDeriv.x * pathDeriv.y)
        secondTerm *= 2 * (pathDeriv.x * pathSecondDeriv.x + pathDeriv.y * pathSecondDeriv.y)
        secondTerm /= (denominator * denominator)
        val secondDeriv = firstTerm - secondTerm

        return if (secondDeriv.isNaN()) 0.0 else secondDeriv
    }
}