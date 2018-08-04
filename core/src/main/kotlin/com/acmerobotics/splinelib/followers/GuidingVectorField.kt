package com.acmerobotics.splinelib.followers

import com.acmerobotics.splinelib.Vector2d
import com.acmerobotics.splinelib.path.Path
import kotlin.math.sign

/**
 * Guiding vector field for effective path following. Described in section III, eq. (9) of
 * [1610.04391.pdf](https://arxiv.org/pdf/1610.04391.pdf). Implementation note: 2D parametric curves are used to
 * describe paths instead of f(x,y) = 0 as described in the paper (which dramatically affects the cross track error
 * calculation).
 *
 * @param path path to follow (interpolator is ignored)
 * @param kN path normal weight (see eq. (9))
 * @param errorMapFunc custom error mapping (see eq. (4))
 */
class GuidingVectorField(
        private val path: Path,
        private val kN: Double,
        private val errorMapFunc: (Double) -> Double = { it }
) {

    class GVFResult(
            val displacement: Double,
            val error: Double,
            val pathPoint: Vector2d,
            val vector: Vector2d
    )

    /**
     * Returns the normalized value of the vector field at the given point along with useful intermediate computations.
     */
    fun getExtended(x: Double, y: Double): GVFResult {
        val point = Vector2d(x, y)
        val projectResult = path.project(Vector2d(x, y))
        val pathPoint = path[projectResult.displacement].pos()
        val tangent = path.deriv(projectResult.displacement).pos()
        val pathToPoint = point - pathPoint
        val orientation = -sign(pathToPoint.x * tangent.y - pathToPoint.y * tangent.x)
        val error = orientation * projectResult.distance
        val normal = tangent.rotated(Math.PI / 2.0)
        val vector = tangent - normal * kN * errorMapFunc(error)
        return GVFResult(projectResult.displacement, error, pathPoint, vector / vector.norm())
    }

    /**
     * Returns the normalized value of the vector field at the given point.
     */
    operator fun get(x: Double, y: Double) = getExtended(x, y).vector
}