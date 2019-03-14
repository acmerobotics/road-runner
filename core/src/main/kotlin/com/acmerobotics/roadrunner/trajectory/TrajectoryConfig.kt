package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * Basic trajectory configuration intended for serialization. Intentionally more simplistic and less flexible than
 * [TrajectoryBuilder].
 *
 * @param poses poses
 * @param constraints constraints
 * @param resolution resolution used for path-based segments (see [PathTrajectorySegment])
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
            if (poses.isEmpty()) {
                Trajectory()
            } else {
                val builder = TrajectoryBuilder(poses.first(), constraints, resolution)
                for (i in 1 until poses.size) {
                    val startPose = poses[i - 1]
                    val endPose = poses[i]
                    if (abs(startPose.x - endPose.x) < 1e-2 && abs(startPose.y - endPose.y) < 1e-2) {
                        // this is probably a turn
                        builder.turnTo(endPose.heading)
                    } else {
                        builder.beginComposite()
                        val diff = endPose - startPose
                        val dot = Vector2d(
                            cos(endPose.heading),
                            sin(endPose.heading)
                        ) dot diff.pos()
                        val cosAngle = dot / diff.pos().norm()

                        builder.setReversed(cosAngle < 0)

                        if (abs(startPose.heading - endPose.heading) < 1e-2 && abs(1 - abs(cosAngle)) < 1e-2) {
                            // this is probably a line
                            builder.lineTo(endPose.pos())
                        } else {
                            // this is probably a spline
                            builder.splineTo(endPose)
                        }
                    }
                }
                builder.build()
            }
}
