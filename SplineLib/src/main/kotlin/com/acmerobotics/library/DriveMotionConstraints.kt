package com.acmerobotics.library

import kotlin.math.abs
import kotlin.math.sqrt

class DriveMotionConstraints(
    val maximumVelocity: Double,
    val maximumAcceleration: Double,
    val maximumAngularVelocity: Double = Double.NaN,
    val maximumCentripetalAcceleration: Double = Double.NaN
) : PathMotionConstraints {
    override fun maximumVelocity(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double {
        val maximumVelocities = mutableListOf(maximumVelocity)

        if (!maximumAngularVelocity.isNaN()) {
            val angularDeriv = poseDeriv.heading
            if (angularDeriv != 0.0) {
                maximumVelocities.add(abs(maximumAngularVelocity / angularDeriv))
            }
        }

        if (!maximumCentripetalAcceleration.isNaN()) {
            val curvature = MathUtil.curvature(poseDeriv.pos(), poseSecondDeriv.pos())
            maximumVelocities.add(sqrt(maximumCentripetalAcceleration / curvature))
        }

        return maximumVelocities.min() ?: 0.0
    }

    override fun maximumAcceleration(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d) = maximumAcceleration
}
