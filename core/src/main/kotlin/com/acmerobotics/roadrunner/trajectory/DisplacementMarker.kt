package com.acmerobotics.roadrunner.trajectory

/**
 * Trajectory marker that is triggered when the specified displacement passes. The exact time is determined at
 * generation-time by calling [displacement] with the path length.
 */
data class DisplacementMarker(val displacement: (Double) -> Double, val callback: MarkerCallback)
