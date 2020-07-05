package com.acmerobotics.roadrunner.profile

import kotlin.math.max
import kotlin.math.min

/**
 * Motion profile parameterized by displacement composed of displacement segments.
 * Note: Many of the methods assume that displacement is monotonically increasing and starts at zero. Consequently,
 * reversed, overshoot, and undershoot profiles are not directly supported.
 *
 * @param segments profile displacement segments
 */
class DisplacementProfile(segments: List<DisplacementSegment>) {
    internal val segments: MutableList<DisplacementSegment> = segments.toMutableList()

    /**
     * Returns the [DisplacementState] at displacement [x] (starting at zero.)
     */
    operator fun get(x: Double): DisplacementState {
        var remainingDisplacement = max(0.0, min(x, length()))
        for (segment in segments) {
            if (remainingDisplacement <= segment.dx) {
                return segment[remainingDisplacement]
            }
            remainingDisplacement -= segment.dx
        }
        return segments.lastOrNull()?.end() ?: DisplacementState(0.0)
    }

    /**
     * Returns the length of the displacement profile.
     */
    fun length() = segments.sumByDouble { it.dx }

    /**
     * Returns the duration of the displacement profile.
     */
    fun duration() = segments.sumByDouble { it.duration() }

    /**
     * Returns a reversed version of the displacement profile.
     */
    fun reversed() = DisplacementProfile(segments.map { it.reversed() }.reversed())

    /**
     * Returns the start [DisplacementState].
     */
    fun start() = get(0.0)

    /**
     * Returns the end [DisplacementState].
     */
    fun end() = get(length())
}
