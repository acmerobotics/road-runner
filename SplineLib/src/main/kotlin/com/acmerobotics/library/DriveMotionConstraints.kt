package com.acmerobotics.library

import kotlin.math.abs
import kotlin.math.sqrt

class DriveMotionConstraints(private val path: HolonomicPath): MotionConstraints {
    override fun maximumVelocity(displacement: Double): Double {
        val maximumVelocities = mutableListOf(path.motionConstraints.maximumVelocity)

        if (path.motionConstraints.maximumAngularVelocity.isFinite()) {
            val angularDeriv = path.deriv(displacement).heading
            if (angularDeriv != 0.0) {
                maximumVelocities.add(abs(path.motionConstraints.maximumAngularVelocity / angularDeriv))
            }
        }

        if (path.motionConstraints.maximumCentripetalAcceleration.isFinite()) {
            val curvature = path.curvature(displacement)
            maximumVelocities.add(sqrt(path.motionConstraints.maximumCentripetalAcceleration / curvature))
        }

        return maximumVelocities.min() ?: 0.0
    }

    override fun maximumAcceleration(displacement: Double) = path.motionConstraints.maximumAcceleration
}