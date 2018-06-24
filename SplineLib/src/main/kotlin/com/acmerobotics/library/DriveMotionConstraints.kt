package com.acmerobotics.library

import kotlin.math.sqrt

open class DriveMotionConstraints(
    val maximumVelocity: Double,
    val maximumAcceleration: Double,
    val path: HolonomicPath? = null,
    val maximumAngularVelocity: Double = Double.POSITIVE_INFINITY,
    val maximumAngularAcceleration: Double = Double.POSITIVE_INFINITY, // TODO: is this possible within this framework?
    val maximumCentripetalAcceleration: Double = Double.POSITIVE_INFINITY
): MotionConstraints {
    override fun maximumVelocity(displacement: Double): Double {
        val maximumVelocities = mutableListOf(maximumVelocity)
        if (path != null) {
            val angularDeriv = path.deriv(displacement).heading
            maximumVelocities.add(maximumAngularVelocity / angularDeriv)

            val curvature = path.curvature(displacement)
            maximumVelocities.add(sqrt(maximumCentripetalAcceleration / curvature))
        }
        return maximumVelocities.min() ?: 0.0
    }

    override fun maximumAcceleration(displacement: Double): Double {
        val maximumAccelerations = mutableListOf(maximumAcceleration)
        return maximumAccelerations.min() ?: 0.0
    }
}
