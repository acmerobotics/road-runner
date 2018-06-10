package com.acmerobotics.library

object Test {
    @JvmStatic
    fun main(args: Array<String>) {
        val trajectory = TrajectoryGenerator.generateTrajectory(
            BezierSpline(),
            MotionState(0.0, 0.0, 0.0),
            MotionState(20.0, 0.0, 0.0),
            { _, dx  -> Math.min(2.0, dx + 0.5) },
            { _, _ -> 5.0 },
            220
        )
        println(trajectory)
        println(trajectory.last().end())
    }
}
