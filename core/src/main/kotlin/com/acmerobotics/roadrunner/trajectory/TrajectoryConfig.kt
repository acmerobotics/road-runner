package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.Pose2d
import kotlin.math.abs

private const val EPSILON = 1e-2

/**
 * Basic trajectory configuration intended for serialization. Intentionally more simplistic and less flexible than
 * [TrajectoryBuilder].
 *
 * @param poses poses
 * @param constraints constraints
 */
class TrajectoryConfig(val poses: List<Pose2d>, val constraints: DriveConstraints) {

    /**
     * Converts the configuration into a real [Trajectory].
     */
    fun toTrajectory() =
            if (poses.isEmpty()) {
                Trajectory()
            } else {
                val builder = TrajectoryBuilder(poses.first(), constraints)
                for (i in 1 until poses.size) {
                    val startPose = poses[i - 1]
                    val endPose = poses[i]
                    if (abs(startPose.x - endPose.x) < EPSILON && abs(startPose.y - endPose.y) < EPSILON) {
                        // this is probably a turn
                        builder.turnTo(endPose.heading)
                    } else {
                        builder.beginComposite()
                        val diff = endPose - startPose
                        val cosAngle = (Math.cos(endPose.heading) * diff.x + Math.sin(endPose.heading) * diff.y) / diff.pos().norm()

                        builder.setReversed(cosAngle < 0)

                        if (abs(startPose.heading - endPose.heading) < EPSILON && abs(1 - abs(cosAngle)) < EPSILON) {
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