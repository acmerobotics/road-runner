package com.acmerobotics.roadrunner.profile

import kotlin.math.max
import kotlin.math.min

/**
 * Trapezoidal motion profile composed of motion segments.
 *
 * @param segments profile motion segments
 */
class MotionProfile(segments: List<MotionSegment>) {
    private val segments: MutableList<MotionSegment> = segments.toMutableList()
    private val start: MotionState = segments.firstOrNull()?.start ?: MotionState(0.0, 0.0, 0.0)

    // TODO: is there a better way to do this?
    // I like the public interface this presents but it's quite ugly internally
    // maybe put the secondary constructor and append control stuff in a builder
    init {
        if (segments.size == 1 && segments[0].dt == 0.0) {
            this.segments.clear()
        }
    }

    constructor(start: MotionState) : this(listOf(MotionSegment(start, 0.0)))

    /**
     * Returns the [MotionState] at time [t].
     */
    operator fun get(t: Double): MotionState {
        var remainingTime = max(0.0, min(t, duration()))
        for (segment in segments) {
            if (remainingTime <= segment.dt) {
                return segment[remainingTime]
            }
            remainingTime -= segment.dt
        }
        return segments.lastOrNull()?.end() ?: start
    }

    fun getSegment(i: Int) = segments[i]

    /**
     * Returns the duration of the motion profile.
     */
    fun duration() = segments.map { it.dt }.sum()

    /**
     * Returns a reversed version of the motion profile.
     */
    fun reversed() = MotionProfile(segments.map { it.reversed() }.reversed())

    /**
     * Returns the end [MotionState].
     */
    fun end() = get(duration())

    fun appendJerkControl(jerk: Double, dt: Double) {
        val endState = end()
        segments.add(MotionSegment(MotionState(endState.x, endState.v, endState.a, jerk), dt))
    }
}