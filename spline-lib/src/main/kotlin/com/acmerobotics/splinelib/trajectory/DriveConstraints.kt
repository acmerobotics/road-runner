package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Pose2d
import com.acmerobotics.splinelib.util.MathUtil
import kotlin.math.abs
import kotlin.math.sqrt

open class DriveConstraints(
    val maximumVelocity: Double,
    val maximumAcceleration: Double,
    val maximumAngularVelocity: Double,
    val maximumAngularAcceleration: Double,
    val maximumCentripetalAcceleration: Double = Double.NaN
) : TrajectoryConstraints {
    override fun maximumVelocity(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double {
        val maximumVelocities = mutableListOf(maximumVelocity)

        if (poseDeriv.heading != 0.0) {
            maximumVelocities.add(abs(maximumAngularVelocity / poseDeriv.heading))
        }

        if (!maximumCentripetalAcceleration.isNaN()) {
            val curvature = MathUtil.curvature(poseDeriv.pos(), poseSecondDeriv.pos())
            maximumVelocities.add(sqrt(maximumCentripetalAcceleration / curvature))
        }

        return maximumVelocities.min() ?: 0.0
    }

    override fun maximumAcceleration(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double {
        val maximumAccelerations = mutableListOf(maximumAcceleration)

        if (poseDeriv.heading != 0.0) {
            val maximumVelocity = maximumVelocity(pose, poseDeriv, poseSecondDeriv)

            maximumAccelerations.add(abs(
                (maximumAngularAcceleration - poseSecondDeriv.heading * maximumVelocity * maximumVelocity) / poseDeriv.heading))
        }

        return maximumAccelerations.min() ?: 0.0
    }
}
