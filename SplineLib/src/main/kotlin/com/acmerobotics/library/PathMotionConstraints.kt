package com.acmerobotics.library

// TODO: I hate that this exists
// Ideally, constraint specific is done at the trajectory level, not the path level
data class PathMotionConstraints(
    val maximumVelocity: Double,
    val maximumAcceleration: Double,
    val maximumAngularVelocity: Double = Double.POSITIVE_INFINITY,
    val maximumCentripetalAcceleration: Double = Double.POSITIVE_INFINITY
)
