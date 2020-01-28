package com.acmerobotics.roadrunner.trajectory

/**
 * Trajectory marker that is triggered when the specified time passes. The exact time is determined at generation-time
 * by calling [time] with the profile duration.
 */
data class TemporalMarker(val time: (Double) -> Double, val callback: MarkerCallback)
