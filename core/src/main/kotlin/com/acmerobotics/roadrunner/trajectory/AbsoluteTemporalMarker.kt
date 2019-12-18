package com.acmerobotics.roadrunner.trajectory

/**
 * Trajectory marker that is triggered when the specified time passes.
 */
data class AbsoluteTemporalMarker(val time: Double, val callback: () -> Unit)
