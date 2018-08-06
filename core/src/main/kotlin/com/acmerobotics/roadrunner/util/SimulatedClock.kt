package com.acmerobotics.roadrunner.util

class SimulatedClock : NanoClock {
    var time: Double = 0.0

    override fun seconds() = time
}