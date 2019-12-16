package com.acmerobotics.roadrunner.trajectory.config

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.PathBuilderException
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.PI

/**
 * Legacy trajectory configuration intended for serialization. Intentionally more simplistic and less flexible than
 * [TrajectoryBuilder].
 *
 * @param poses poses
 * @param constraints constraints
 * @param resolution resolution used for path-based segments (see [Trajectory])
 */
class TrajectoryConfigV1 @JvmOverloads constructor(
    val poses: List<Pose2d>,
    val constraints: DriveConstraints,
    val resolution: Double = 0.25
) : TrajectoryConfig {

    /**
     * Converts the configuration into a [TrajectoryBuilder].
     */
    @Suppress("NestedBlockDepth")
    fun toTrajectoryBuilder() =
            if (poses.size < 2) {
                null
            } else {
                try {
                    val firstPose = poses.first()
                    val secondPose = poses.drop(1).first()
                    val reversed = (secondPose - firstPose).vec() angleBetween secondPose.headingVec() > PI / 4
                    val builder = TrajectoryBuilder(
                        firstPose,
                        constraints = constraints,
                        resolution = resolution,
                        reversed = reversed
                    )
                    for (i in 1 until poses.size) {
                        val startPose = poses[i - 1]
                        val endPose = poses[i]

                        if (startPose.heading epsilonEquals endPose.heading &&
                            startPose.heading epsilonEquals (endPose - startPose).vec().angle()) {
                            // this is probably a line
                            builder.lineTo(endPose.vec())
                        } else {
                            // this is probably a spline
                            builder.splineTo(endPose)
                        }
                    }
                    builder
                } catch (e: PathBuilderException) {
                    null
                }
            }

    /**
     * Convenience wrapper for [toTrajectoryBuilder].
     */
    fun toTrajectory() = toTrajectoryBuilder()?.build()
}
