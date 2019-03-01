package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.util.NanoClock

/**
 * Simulated clock with user-specified time.
 */
class SimulatedClock : NanoClock() {
    var time: Double = 0.0

    override fun seconds() = time
}
