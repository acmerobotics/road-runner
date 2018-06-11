package com.acmerobotics.library

import java.io.File

object Test {
    @JvmStatic
    fun main(args: Array<String>) {
        val profile = MotionProfileGenerator.generateMotionProfile(
            MotionState(5.0, 0.0, 0.0),
            MotionState(25.0, 5.0, 0.0),
            { _ -> 10.0 },
            { _ -> 5.0 },
            4
        )

        File("profile.csv").printWriter().use { out ->
            out.println("t,x,v,a")
            for (i in 0..1000) {
                val t = i / 1000.0 * profile.duration()
                val state = profile[t]
                out.format("%f,%f,%f,%f\n", t, state.x, state.v, state.a)
            }
        }
    }
}
