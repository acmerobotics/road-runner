package com.acmerobotics.splinelib.followers

import com.acmerobotics.splinelib.Vector2d
import com.acmerobotics.splinelib.path.Path
import kotlin.math.sign

class GuidingVectorField(private val path: Path, private val kN: Double, private val errorMapFunc: (Double) -> Double = { it }) {
    operator fun get(x: Double, y: Double): Vector2d {
        val point = Vector2d(x, y)
        val projectResult = path.project(Vector2d(x, y))
        val pathPoint = path[projectResult.displacement].pos()
        val tangent = path.deriv(projectResult.displacement).pos()
        val pathToPoint = point - pathPoint
        val orientation = -sign(pathToPoint.x * tangent.y - pathToPoint.y * tangent.x)
        val error = errorMapFunc(orientation * projectResult.distance)
        val normal = tangent.rotated(Math.PI / 2.0)
        val vector = tangent - normal * kN * error
        return vector / vector.norm()
    }
}