package com.acmerobotics.roadrunner.util

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.times
import com.acmerobotics.roadrunner.path.Path
import kotlin.math.*

/**
 * Guiding vector field for effective path following described in section III, eq. (9) of
 * [1610.04391.pdf](https://arxiv.org/pdf/1610.04391.pdf). Implementation note: 2D parametric curves are used to
 * describe paths instead of implicit curves of the form f(x,y) = 0 as described in the paper (which dramatically
 * affects the cross track error calculation).
 *
 * @param path path to follow (interpolator is ignored)
 * @param kN path normal weight (see eq. (9))
 * @param errorMap custom error mapping (see eq. (4))
 */
class GuidingVectorField(
    private val path: Path,
    private val kN: Double,
    private val errorMap: ErrorMap = ErrorMap.Linear()
) {

    /**
     * Represents the error mapping function and its derivative (see eq. (4))
     * Error maps describe functions that convert cross-track error into the normal vector's weight
     * [Graphed on Desmos](https://www.desmos.com/calculator/wy1u493y7k)
     */
    abstract class ErrorMap {

        /**
         * The linear error mapping function.
         * f(x) = x and f'(x) = 1.0
         */
        class Linear : ErrorMap() {
            override fun get(error: Double) = error
            override fun deriv(error: Double) = 1.0
        }

        /**
         * The damped power error mapping function.
         * Where p = power >= 1, b = horizontalStretch,
         * f(x) = |x/b|^p*sign(x)/(1+|x/b|^p) and f'(x) = p/b*|x/b|^(p-1)/(1+|x/b|^p)^2
         *
         * @param power the function power >= 1
         * @param horizontalStretch the compression/stretch transformation of the input value
         */
        class DampedPower(private val power: Double, private val horizontalStretch: Double) : ErrorMap() {
            override fun get(error: Double) =
                    abs(error / horizontalStretch).pow(power) * sign(error) /
                            (1.0 + abs(error / horizontalStretch).pow(power))
            override fun deriv(error: Double) =
                    power / horizontalStretch * abs(error / horizontalStretch).pow(power - 1.0) /
                            (1.0 + abs(error / horizontalStretch).pow(power)) *
                            (1.0 + abs(error / horizontalStretch).pow(power))
        }

        /**
         * The arc tangent error mapping function.
         * Where p = power >= 1, b = horizontalStretch,
         * f(x) = atan(|x/b|^p*sign(x))/(pi/2) and f'(x) = p|x/b|^(p-1)/(pi/2*b*(1+|x/b|^(2p)))
         *
         * @param power the function power >= 1
         * @param horizontalStretch the compression/stretch transformation of the input value
         */
        class Atan(private val power: Double, private val horizontalStretch: Double) : ErrorMap() {
            override fun get(error: Double) = atan(abs(error / horizontalStretch).pow(power) * sign(error)) /
                    (PI / 2.0)
            override fun deriv(error: Double) = power * abs(error / horizontalStretch).pow(power - 1.0) /
                    (PI / 2.0 * horizontalStretch * (1.0 + abs(error / horizontalStretch).pow(2.0 * power)))
        }

        /**
         * Gets the value of the error mapping function at [error]
         *
         * @param error the error to be mapped
         */
        abstract operator fun get(error: Double): Double

        /**
         * Gets the derivative of the error mapping function at [error]
         *
         * @param error the error to be mapped
         */
        abstract fun deriv(error: Double): Double
    }

    /**
     * Container for the direction of the GVF and intermediate values used in its computation.
     *
     * @param vector normalized direction vector of the GVF
     * @param deriv derivative of the normalized gvf vector with respect to displacement
     * @param displacementDeriv *path* displacement derivative with respect to displacement based on gvf vector
     * @param error signed cross track error (distance between the path point and the query point)
     */
    data class GVFResult(
        val vector: Vector2d,
        val deriv: Vector2d,
        val displacementDeriv: Double,
        val error: Double
    )

    /**
     * Returns the normalized value of the vector field at the given point along with useful intermediate computations.
     */
    fun getExtended(point: Vector2d, displacement: Double = path.project(point)): GVFResult {
        val pathPoint = path[displacement].vec()
        val pathDeriv = path.deriv(displacement).vec() // tangent
        val pathSecondDeriv = path.secondDeriv(displacement).vec()

        val pathToPoint = point - pathPoint
        val error = pathToPoint dot pathDeriv.rotated(PI / 2.0)
        val normal = pathDeriv.rotated(PI / 2.0)
        val vector = pathDeriv - normal * kN * errorMap[error]
        val unitVector = vector.unit()

        val projDeriv = (unitVector dot pathDeriv) / (1.0 - (pathToPoint dot pathSecondDeriv))
        val errorDeriv = errorMap.deriv(error) * (unitVector dot pathDeriv.rotated(PI / 2.0))
        val tangentDeriv = pathSecondDeriv * projDeriv
        val normalDeriv = tangentDeriv.rotated(-PI / 2.0)
        val vectorDeriv = (tangentDeriv - (kN * errorMap.deriv(error) * normalDeriv)) - (kN * errorDeriv * normal)
        val unitVectorDeriv = ((vector dot vector) * vectorDeriv - (vectorDeriv dot vector) * vector) /
                (vector.norm() * vector.norm() * vector.norm())

        return GVFResult(
                unitVector,
                unitVectorDeriv,
                projDeriv,
                error
        )
    }

    /**
     * Returns the normalized value of the vector field at the given point.
     */
    operator fun get(point: Vector2d) = getExtended(point).vector

    /**
     * Returns the derivative of the normalized vector field at the given point.
     */
    fun deriv(point: Vector2d) = getExtended(point).deriv
}
