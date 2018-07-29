package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Pose2d
import kotlin.math.abs

class TrajectoryConfig(val poses: List<Pose2d>, val constraints: DriveConstraints) {
    companion object {
        const val EPSILON = 1e-2
    }

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