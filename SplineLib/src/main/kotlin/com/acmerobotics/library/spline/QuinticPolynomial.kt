package com.acmerobotics.library.spline

import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils

class QuinticPolynomial(start: Double, startDeriv: Double, startSecondDeriv: Double,
                        end: Double, endDeriv: Double, endSecondDeriv: Double) {
    val a: Double
    val b: Double
    val c: Double
    val d: Double
    val e: Double
    val f: Double

    companion object {
        private val COEFF_MATRIX = MatrixUtils.createRealMatrix(
            arrayOf(
                doubleArrayOf(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                doubleArrayOf(0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
                doubleArrayOf(0.0, 0.0, 2.0, 0.0, 0.0, 0.0),
                doubleArrayOf(1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
                doubleArrayOf(0.0, 1.0, 2.0, 3.0, 4.0, 5.0),
                doubleArrayOf(0.0, 0.0, 2.0, 6.0, 12.0, 20.0)
            )
        )
    }

    init {
        val target =
            MatrixUtils.createRealMatrix(arrayOf(doubleArrayOf(
                start, startDeriv, startSecondDeriv, end, endDeriv, endSecondDeriv
            ))).transpose()

        val solver = LUDecomposition(COEFF_MATRIX).solver
        val coeff = solver.solve(target)

        f = coeff.getEntry(0, 0)
        e = coeff.getEntry(1, 0)
        d = coeff.getEntry(2, 0)
        c = coeff.getEntry(3, 0)
        b = coeff.getEntry(4, 0)
        a = coeff.getEntry(5, 0)
    }
    
    operator fun get(t: Double) = (a*t + b) * (t*t*t*t) + c * (t*t*t) + d * (t*t) + e * t + f
    
    fun deriv(t: Double) = (5*a*t + 4*b) * (t*t*t) + (3*c*t + 2*d) * t + e
    
    fun secondDeriv(t: Double) = (20*a*t + 12*b) * (t*t) + 6*c * t + 2*d

    fun thirdDeriv(t: Double) = (60*a*t + 24*b) * t + 6*c

    override fun toString() = "$a*t^5+$b*t^4+$c*t^3+$d*t^2+$e*t+$f"
}