package com.acmerobotics.library.trajectory

import com.acmerobotics.library.MathUtil
import com.acmerobotics.library.Pose2d
import kotlin.math.abs
import kotlin.math.sqrt

class DriveMotionConstraints(
    val maximumVelocity: Double,
    val maximumAcceleration: Double,
    val maximumAngularVelocity: Double = Double.NaN,
    val maximumAngularAcceleration: Double = Double.NaN,
    val maximumCentripetalAcceleration: Double = Double.NaN
) : PathMotionConstraints {
    override fun maximumVelocity(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double {
        val maximumVelocities = mutableListOf<Double>()

        if (poseDeriv.x != 0.0) {
            maximumVelocities.add(abs(maximumVelocity / poseDeriv.x))
        }

        if (poseDeriv.y != 0.0) {
            maximumVelocities.add(abs(maximumVelocity / poseDeriv.y))
        }

        if (!maximumAngularVelocity.isNaN() && poseDeriv.heading != 0.0) {
            maximumVelocities.add(abs(maximumAngularVelocity / poseDeriv.heading))
        }

        if (!maximumCentripetalAcceleration.isNaN()) {
            val curvature = MathUtil.curvature(poseDeriv.pos(), poseSecondDeriv.pos())
            maximumVelocities.add(sqrt(maximumCentripetalAcceleration / curvature))
        }

        return maximumVelocities.min() ?: 0.0
    }

    override fun maximumAcceleration(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double {
        val maximumAccelerations = mutableListOf<Double>()

        val maximumVelocity = maximumVelocity(pose, poseDeriv, poseSecondDeriv)

        if (poseDeriv.x != 0.0) {
            maximumAccelerations.add(abs(
                (maximumAcceleration - poseSecondDeriv.x * maximumVelocity * maximumVelocity) / poseDeriv.x))
        }

        if (poseDeriv.y != 0.0) {
            maximumAccelerations.add(abs(
                (maximumAcceleration - poseSecondDeriv.y * maximumVelocity * maximumVelocity) / poseDeriv.y))
        }

        if (!maximumAngularAcceleration.isNaN() && poseDeriv.heading != 0.0) {
            maximumAccelerations.add(abs(
                (maximumAngularAcceleration - poseSecondDeriv.heading * maximumVelocity * maximumVelocity) / poseDeriv.heading))
        }

        return maximumAccelerations.min() ?: 0.0
    }
}
