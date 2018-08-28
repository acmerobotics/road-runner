package com.acmerobotics.roadrunner.profile

class MotionProfileBuilder(start: MotionState) {
    private var currentState = start
    private val segments = mutableListOf<MotionSegment>()

    fun appendJerkControl(jerk: Double, dt: Double): MotionProfileBuilder {
        val segment = MotionSegment(MotionState(currentState.x, currentState.v, currentState.a, jerk), dt)
        segments.add(segment)
        currentState = segment.end()
        return this
    }

    fun appendAccelerationControl(accel: Double, dt: Double): MotionProfileBuilder {
        val segment = MotionSegment(MotionState(currentState.x, currentState.v, accel), dt)
        segments.add(segment)
        currentState = segment.end()
        return this
    }

    fun appendProfile(profile: MotionProfile): MotionProfileBuilder {
        for (segment in profile.segments) {
            if (segment.start.j.isNaN()) {
                // constant acceleration
                appendAccelerationControl(segment.start.a, segment.dt)
            } else {
                // constant jerk
                appendJerkControl(segment.start.j, segment.dt)
            }
        }
        return this
    }

    fun build() = MotionProfile(segments)
}