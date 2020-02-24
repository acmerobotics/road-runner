package com.acmerobotics.roadrunner.trajectory.config

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.epsilonEquals

/**
 * Configuration describing a basic trajectory (a simpler frontend alternative to [BaseTrajectoryBuilder]).
 */
data class TrajectoryConfig(
    val startPose: Pose2d,
    val startHeading: Double?,
    val steps: List<Step>,
    val resolution: Double
) {
    val version = 2

    /**
     * Heading interpolation for a specific trajectory configuration step.
     */
    enum class HeadingInterpolationType {
        TANGENT,
        CONSTANT,
        LINEAR,
        SPLINE
    }

    /**
     * Description of a single segment of a composite trajectory.
     */
    data class Step @JvmOverloads constructor(
        val pose: Pose2d,
        val heading: Double? = null,
        val interpolationType: HeadingInterpolationType
    )

    @Suppress("ComplexMethod")
    fun toTrajectoryBuilder(groupConfig: TrajectoryGroupConfig): TrajectoryBuilder? {
        val builder = TrajectoryBuilder(
            startPose,
            startHeading ?: startPose.heading,
            groupConfig.constraints,
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
