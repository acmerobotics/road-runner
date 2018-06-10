package com.acmerobotics.library

import kotlin.math.abs
import kotlin.math.sqrt

object TrajectoryGenerator {
    fun generateTrajectory(
        path: BezierSpline,
        start: MotionState,
        goal: MotionState,
        velocityConstraints: (path: BezierSpline, displacement: Double) -> Double,
        accelerationConstraints: (path: BezierSpline, displacement: Double) -> Double,
        resolution: Int = 250,
        epsilon: Double = 1e-6
    ): List<MotionSegment> {
        val length = path.length()
        val dx = length / resolution

        val forwardStates = forwardPass(
            path,
            start,
            (0 until resolution).map { it * dx },
            velocityConstraints,
            accelerationConstraints,
            dx
        )
        val backwardStates = forwardPass(
            path,
            goal,
            (resolution downTo 1).map { it * dx },
            velocityConstraints,
            accelerationConstraints,
            dx
        ).reversed().map { (motionState, dx) ->
            Pair(MotionState(motionState.x, motionState.v, -motionState.a).afterDisplacement(-dx), dx)
        }

        val finalStates = mutableListOf<Pair<MotionState, Double>>()

        var i = 0
        var j = 0
        while (i < forwardStates.size) {
            val (forwardStartState, forwardDx) = forwardStates[i]
            val (backwardStartState, backwardDx) = backwardStates[j]

            val forwardEndState = forwardStartState.afterDisplacement(forwardDx)
            val backwardEndState = backwardStartState.afterDisplacement(backwardDx)

            if (forwardStartState.v <= backwardStartState.v) {
                if (forwardEndState.v <= backwardEndState.v) {
                    finalStates.add(forwardStates[i])
                } else {
                    val intersection = intersection(forwardStartState, backwardStartState)
                    finalStates.add(Pair(forwardStartState, intersection))
                    finalStates.add(
                        Pair(
                            backwardStartState.afterDisplacement(intersection),
                            backwardDx - intersection
                        )
                    )
                }
            } else {
                if (forwardEndState.v >= backwardEndState.v) {
                    finalStates.add(backwardStates[j])
                } else {
                    val intersection = intersection(forwardStartState, backwardStartState)
                    finalStates.add(Pair(backwardStartState, intersection))
                    finalStates.add(
                        Pair(
                            forwardStartState.afterDisplacement(intersection),
                            forwardDx - intersection
                        )
                    )
                }
            }

            when {
                abs(forwardEndState.x - backwardEndState.x) < epsilon -> {
                    i++
                    j++
                }
                forwardEndState.x < backwardEndState.x -> i++
                else -> j++
            }
        }

        val motionSegments = mutableListOf<MotionSegment>()
        for ((state, stateDx) in finalStates) {
            val dt = (sqrt(state.v * state.v + 2 * state.a * stateDx) - state.v) / state.a
            motionSegments.add(MotionSegment(state, dt))
        }

        return motionSegments
    }

    private fun forwardPass(
        path: BezierSpline,
        start: MotionState,
        displacements: List<Double>,
        velocityConstraints: (path: BezierSpline, displacement: Double) -> Double,
        accelerationConstraints: (path: BezierSpline, displacement: Double) -> Double,
        dx: Double
    ): List<Pair<MotionState, Double>> {
        val forwardStates = mutableListOf<Pair<MotionState, Double>>()

        var lastState = start
        for (displacement in displacements) {
            val maximumVelocity = velocityConstraints(path, displacement)
            val maximumAcceleration = accelerationConstraints(path, displacement)

            lastState = if (lastState.v >= maximumVelocity) {
                val state = MotionState(displacement, maximumVelocity, 0.0)
                forwardStates.add(Pair(state, dx))
                state
            } else {
                val desiredVelocity = sqrt(lastState.v * lastState.v + 2 * maximumAcceleration * dx)
                if (desiredVelocity <= maximumVelocity) {
                    val state = MotionState(displacement, lastState.v, maximumAcceleration)
                    forwardStates.add(Pair(state, dx))
                    state
                } else {
                    val accelDx =
                        (maximumVelocity * maximumVelocity - lastState.v * lastState.v) / (2 * maximumAcceleration)
                    val accelState = MotionState(displacement, lastState.v, maximumAcceleration)
                    val coastState = MotionState(displacement + dx, maximumVelocity, 0.0)
                    forwardStates.add(Pair(accelState, accelDx))
                    forwardStates.add(Pair(coastState, dx - accelDx))
                    coastState
                }
            }.afterDisplacement(dx)
        }

        return forwardStates
    }

    private fun intersection(state1: MotionState, state2: MotionState): Double {
        return (state1.v * state1.v - state2.v * state2.v) / (2 * state2.a - 2 * state1.a)
    }
}