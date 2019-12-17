package com.acmerobotics.roadrunner.trajectory.config

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.util.epsilonEquals

class TrajectoryConfig @JvmOverloads constructor(
    var startPose: Pose2d,
    var startHeading: Double?,
    var steps: List<Step>,
    var resolution: Double,
    var reversed: Boolean
) {
    private val version = 2

    enum class HeadingInterpolationType {
        TANGENT,
        CONSTANT,
        LINEAR,
        SPLINE
    }

    class Step @JvmOverloads constructor(
        val pose: Pose2d,
        val interpolationType: HeadingInterpolationType,
        val heading: Double? = null
    )

    fun toTrajectoryBuilder(groupConfig: TrajectoryGroupConfig): TrajectoryBuilder? {
        val builder = TrajectoryBuilder(
            startPose,
            startHeading ?: startPose.heading,
            groupConfig.specificConstraints,
            resolution = resolution,
            reversed = reversed
        )

        for (step in steps) {
            val currentPose = builder.currentPose!!
            val line = currentPose.heading epsilonEquals step.pose.heading &&
                currentPose.heading epsilonEquals (step.pose - currentPose).vec().angle()
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
