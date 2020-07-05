package com.acmerobotics.roadrunner.util

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.times
import com.acmerobotics.roadrunner.path.Path
import kotlin.math.PI
import kotlin.math.atan

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
         * The cubic error mapping function.
         * Where a = cubeCoefficient, f(x) = ax^3 + x and f'(x) = a/3x^2 + 1
         *
         * @param cubeCoefficient the third-order term coefficient
         */
        class Cubic(private val cubeCoefficient: Double = 0.01) : ErrorMap() {
            override fun get(error: Double) = cubeCoefficient * error * error * error + error
            override fun deriv(error: Double) = cubeCoefficient / 3.0 * error * error + 1.0
        }

        /**
         * The arc tangent error mapping function.
         * Where b = horizontalStretch, f(x) = atan(b * x) and f'(x) = b / (1 + x^2)
         *
         * @param horizontalStretch the compression/stretch transformation of the input value
         */
        class Atan(private val horizontalStretch: Double = 1.0) : ErrorMap() {
            override fun get(error: Double) = atan(horizontalStretch * error)
            override fun deriv(error: Double) = horizontalStretch / (1 + error * error)
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
