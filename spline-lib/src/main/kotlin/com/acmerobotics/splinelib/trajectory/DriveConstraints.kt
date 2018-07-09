package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Pose2d
import com.acmerobotics.splinelib.util.MathUtil
import kotlin.math.sqrt

open class DriveConstraints(
        @JvmField val maximumVelocity: Double,
        @JvmField val maximumAcceleration: Double,
        @JvmField val maximumAngularVelocity: Double,
        @JvmField val maximumAngularAcceleration: Double,
        @JvmField val maximumCentripetalAcceleration: Double
) : TrajectoryConstraints {
    override fun maximumVelocity(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double {
        val maximumVelocities = mutableListOf(maximumVelocity)

        // TODO: figure out why this makes splines significantly slower
//        if (poseDeriv.heading != 0.0) {
//            maximumVelocities.add(abs(maximumAngularVelocity / poseDeriv.heading))
//        }

        if (!maximumCentripetalAcceleration.isNaN()) {
            val curvature = MathUtil.curvature(poseDeriv.pos(), poseSecondDeriv.pos())
            maximumVelocities.add(sqrt(maximumCentripetalAcceleration / curvature))
        }

        return maximumVelocities.min() ?: 0.0
    }

    override fun maximumAcceleration(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double {
        val maximumAccelerations = mutableListOf(maximumAcceleration)

//        if (poseDeriv.heading != 0.0) {
//            val maximumVelocity = maximumVelocity(pose, poseDeriv, poseSecondDeriv)
//
//            maximumAccelerations.add(abs(
//                (maximumAngularAcceleration - poseSecondDeriv.heading * maximumVelocity * maximumVelocity) / poseDeriv.heading))
//        }

        return maximumAccelerations.min() ?: 0.0
    }
}
