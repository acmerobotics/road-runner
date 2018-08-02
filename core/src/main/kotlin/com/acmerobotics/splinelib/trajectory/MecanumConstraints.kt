package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Pose2d
import kotlin.math.abs
import kotlin.math.min

/**
 * Mecanum-specific drive constraints that also limit maximum wheel velocities.
 *
 * @param baseConstraints base drive constraints
 * @param trackWidth track width
 * @param wheelBase wheel base
 */
class MecanumConstraints(
        baseConstraints: DriveConstraints,
        trackWidth: Double,
        wheelBase: Double = trackWidth
) : DriveConstraints(
        baseConstraints.maximumVelocity,
        baseConstraints.maximumAcceleration,
        baseConstraints.maximumAngularVelocity,
        baseConstraints.maximumAngularAcceleration
) {
    private val k = (trackWidth + wheelBase) / 2.0
    // TODO: verify this is actually correct
    override fun maximumVelocity(pose: Pose2d, poseDeriv: Pose2d, poseSecondDeriv: Pose2d): Double {
        val robotPositionDeriv = poseDeriv.pos().rotated(-pose.heading)

        val maxWheelVel = maximumVelocity / (abs(robotPositionDeriv.x) + abs(robotPositionDeriv.y) + abs(k * poseDeriv.heading))

        return min(super.maximumVelocity(pose, poseDeriv, poseSecondDeriv), maxWheelVel)
    }

}