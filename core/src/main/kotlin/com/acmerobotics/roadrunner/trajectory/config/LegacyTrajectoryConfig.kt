package com.acmerobotics.roadrunner.trajectory.config

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.util.Angle
import kotlin.math.PI

/**
 * Legacy trajectory configuration intended for serialization. Intentionally more simplistic and less flexible than
 * [TrajectoryBuilder].
 *
 * @param poses poses
 * @param constraints constraints
 * @param resolution resolution used for path-based segments (see [Trajectory])
 */
class LegacyTrajectoryConfig @JvmOverloads constructor(
    val poses: List<Pose2d>,
    val constraints: DriveConstraints,
    val resolution: Double = 0.25
) {

    /**
     * Converts the configuration into a [TrajectoryBuilder].
     */
    @Suppress("NestedBlockDepth")
    fun toTrajectoryConfig() =
        if (poses.size < 2) {
            null
        } else {
            val firstPose = poses.first()
            val secondPose = poses.drop(1).first()
            val startHeading = if ((secondPose - firstPose).vec() angleBetween secondPose.headingVec() > PI / 4) {
                Angle.norm(firstPose.heading + PI)
            } else {
                null
            }
            val steps = poses.drop(1).map {
                TrajectoryConfig.Step(it, TrajectoryConfig.HeadingInterpolationType.TANGENT)
            }
            TrajectoryConfig(firstPose, startHeading = startHeading, steps = steps, resolution = resolution)
        }
}
