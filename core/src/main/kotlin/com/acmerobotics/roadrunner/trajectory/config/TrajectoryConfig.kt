package com.acmerobotics.roadrunner.trajectory.config

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.epsilonEquals

class TrajectoryConfig(
    var startPose: Pose2d,
    var startHeading: Double?,
    var steps: List<Step>,
    var resolution: Double
) {
    val version = 2

    enum class HeadingInterpolationType {
        TANGENT,
        CONSTANT,
        LINEAR,
        SPLINE
    }

    class Step @JvmOverloads constructor(
        val pose: Pose2d,
        val heading: Double? = null,
        val interpolationType: HeadingInterpolationType
    )

    fun toTrajectoryBuilder(groupConfig: TrajectoryGroupConfig): TrajectoryBuilder? {
        val builder = TrajectoryBuilder(
            startPose,
            startHeading ?: startPose.heading,
            groupConfig.specificConstraints,
            resolution = resolution
        )

        for (step in steps) {
            val line = Angle.normDelta(startPose.heading - step.pose.heading) epsilonEquals 0.0 &&
                startPose.heading epsilonEquals (step.pose - startPose).vec().angle()
            when (step.interpolationType) {
                HeadingInterpolationType.TANGENT -> if (line) {
                    builder.lineTo(step.pose.vec())
                } else {
                    builder.splineTo(step.pose)
                }
                HeadingInterpolationType.CONSTANT -> if (line) {
                    builder.lineToConstantHeading(step.pose.vec())
                } else {
                    builder.splineToConstantHeading(step.pose)
                }
                HeadingInterpolationType.LINEAR -> if (line) {
                    builder.lineToLinearHeading(step.pose.vec(), step.heading!!)
                } else {
                    builder.splineToLinearHeading(step.pose, step.heading!!)
                }
                HeadingInterpolationType.SPLINE -> if (line) {
                    builder.lineToSplineHeading(step.pose.vec(), step.heading!!)
                } else {
                    builder.splineToSplineHeading(step.pose, step.heading!!)
                }
            }
        }

        return builder
    }

    fun toTrajectory(groupConfig: TrajectoryGroupConfig) = toTrajectoryBuilder(groupConfig)?.build()
}
