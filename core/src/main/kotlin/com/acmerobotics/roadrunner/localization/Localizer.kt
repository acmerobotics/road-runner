package com.acmerobotics.roadrunner.localization

import com.acmerobotics.roadrunner.geometry.Pose2d

/**
 * Generic interface for estimating robot pose over time.
 */
interface Localizer {

    /**
     * Current robot pose estimate.
     */
    var poseEstimate: Pose2d

    /**
     * Completes a single localization update.
     */
    fun update()
}
