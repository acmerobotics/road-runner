package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * Basic trajectory configuration intended for serialization. Intentionally more simplistic and less flexible than
 * [BaseTrajectoryBuilder].
 *
 * @param poses poses
 * @param constraints constraints
 * @param resolution resolution used for path-based segments (see [Trajectory])
 */
class TrajectoryConfig @JvmOverloads constructor(
    val poses: List<Pose2d>,
    val constraints: DriveConstraints,
    val resolution: Double = 0.25
) {

    /**
     * Converts the configuration into a real [Trajectory].
     */
    @Suppress("NestedBlockDepth")
    fun toTrajectory() =
            if (poses.size < 2) {
                null
            } else {
                // TODO: fix reversal
                val builder = TrajectoryBuilder(poses.first(), constraints, resolution = resolution)
                for (i in 1 until poses.size) {
                    val startPose = poses[i - 1]
                    val endPose = poses[i]
                    val diff = endPose - startPose
                    val dot = Vector2d(
                        cos(endPose.heading),
                        sin(endPose.heading)
                    ) dot diff.vec()
                    val cosAngle = dot / diff.vec().norm()

                    if (startPose.heading epsilonEquals endPose.heading && abs(cosAngle) epsilonEquals 1.0) {
                        // this is probably a line
                        builder.lineTo(endPose.vec())
                    } else {
                        // this is probably a spline
                        builder.splineTo(endPose)
                    }
                }
                builder.build()
            }
}
