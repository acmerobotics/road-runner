package com.acmerobotics.roadrunner.profile

/**
 * Segment of a displacement profile.
 *
 * @param start start displacement state
 * @param dx displacement delta
 */
class DisplacementSegment(val start: DisplacementState, val dx: Double) {

    /**
     * Returns the [DisplacementState] at displacement [dx].
     */
    operator fun get(x: Double) = start[x]

    /**
     * Returns the [DisplacementState] at the end of the segment (displacement [dx]).
     */
    fun end() = start[dx]

    /**
     * Returns the duration of the segment. This is based on the [start] state.
     */
    fun duration() = start.getTime(dx)

    /**
     * Returns a reversed version of the segment. Note: it isn't possible to reverse a segment completely so this
     * method only guarantees that the start and end velocities will be swapped.
     */
    fun reversed(): DisplacementSegment {
        val end = end()
        val state = DisplacementState(end.v, -end.a, end.j)
        return DisplacementSegment(state, dx)
    }

    override fun toString() = "($start, $dx)"
}
