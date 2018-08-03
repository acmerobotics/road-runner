package com.acmerobotics.splinelib.followers

import com.acmerobotics.splinelib.Vector2d
import com.acmerobotics.splinelib.path.Path

class GuidingVectorField(private val path: Path, private val kN: Double, private val errorMapFunc: (Double) -> Double = { it }) {
    operator fun get(x: Double, y: Double): Vector2d {
        val s = 0.0 // TODO: find s
        val distanceToPath = 0.0 // TODO: find from the projection
        val error = errorMapFunc(distanceToPath)
        val tangent = path.deriv(s).pos()
        val normal = tangent.rotated(Math.PI / 2.0)
        val vector = tangent - normal * kN * error
        return vector / vector.norm()
    }


}