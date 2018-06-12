package com.acmerobotics.library

import kotlin.math.abs
import kotlin.math.sqrt

object MotionProfileGenerator {
    fun generateSimpleMotionProfile(
        start: MotionState,
        goal: MotionState,
        maximumVelocity: Double,
        maximumAcceleration: Double,
        epsilon: Double = 1e-6
    ): MotionProfile = generateMotionProfile(start, goal, { maximumVelocity }, { maximumAcceleration }, 1, epsilon)

    fun generateMotionProfile(
        start: MotionState,
        goal: MotionState,
        maximumVelocity: (displacement: Double) -> Double,
        maximumAcceleration: (displacement: Double) -> Double,
        resolution: Int = 250,
        epsilon: Double = 1e-6
    ): MotionProfile {
        if (goal.x < start.x) {
            return generateMotionProfile(
                goal,
                start,
                maximumVelocity,
                maximumAcceleration,
                resolution,
                epsilon
            ).reversed()
        }

        val length = goal.x - start.x
        val dx = length / resolution

        val forwardStates = forwardPass(
            MotionState(0.0, start.v, start.a),
            { maximumVelocity(start.x + it) },
            { maximumAcceleration(start.x + it) },
            resolution,
            dx
        ).map { (motionState, dx) -> Pair(MotionState(motionState.x + start.x, motionState.v, motionState.a), dx) }
            .toMutableList()

        val backwardStates = forwardPass(
            MotionState(0.0, goal.v, goal.a),
            { maximumVelocity(goal.x - it) },
            { maximumAcceleration(goal.x - it) },
            resolution,
            dx
        ).map { (motionState, dx) -> Pair(motionState.afterDisplacement(dx), dx) }.map { (motionState, dx) ->
            Pair(
                MotionState(
                    goal.x - motionState.x,
                    motionState.v,
                    -motionState.a
                ), dx
            )
        }.reversed().toMutableList()

        val finalStates = mutableListOf<Pair<MotionState, Double>>()

        var i = 0
        var j = 0
        while (i < forwardStates.size) {
            var (forwardStartState, forwardDx) = forwardStates[i]
            var (backwardStartState, backwardDx) = backwardStates[j]

            if (abs(forwardDx - backwardDx) > epsilon) {
                if (forwardDx < backwardDx) {
                    backwardStates.add(
                        j + 1,
                        Pair(backwardStartState.afterDisplacement(forwardDx), backwardDx - forwardDx)
                    )
                    backwardDx = forwardDx
                } else {
                    forwardStates.add(
                        i + 1,
                        Pair(forwardStartState.afterDisplacement(backwardDx), forwardDx - backwardDx)
                    )
                    forwardDx = backwardDx
                }
            }

            val forwardEndState = forwardStartState.afterDisplacement(forwardDx)
            val backwardEndState = backwardStartState.afterDisplacement(backwardDx)

            if (forwardStartState.v <= backwardStartState.v) {
                if (forwardEndState.v <= backwardEndState.v) {
                    finalStates.add(Pair(forwardStartState, forwardDx))
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
                    finalStates.add(Pair(backwardStartState, backwardDx))
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
            i++
            j++
        }

        val motionSegments = mutableListOf<MotionSegment>()
        for ((state, stateDx) in finalStates) {
            val dt = if (abs(state.a) > epsilon) {
                (sqrt(state.v * state.v + 2 * state.a * stateDx) - state.v) / state.a
            } else {
                stateDx / state.v
            }
            motionSegments.add(MotionSegment(state, dt))
        }

        return MotionProfile(motionSegments)
    }

    private fun forwardPass(
        start: MotionState,
        maximumVelocity: (displacement: Double) -> Double,
        maximumAcceleration: (displacement: Double) -> Double,
        resolution: Int,
        dx: Double
    ): List<Pair<MotionState, Double>> {
        val forwardStates = mutableListOf<Pair<MotionState, Double>>()

        val displacements = (0 until resolution).map { it * dx + start.x }

        var lastState = start
        for (displacement in displacements) {
            val maxVel = maximumVelocity(displacement)
            val maxAccel = maximumAcceleration(displacement)

            lastState = if (lastState.v >= maxVel) {
                val state = MotionState(displacement, maxVel, 0.0)
                forwardStates.add(Pair(state, dx))
                state.afterDisplacement(dx)
            } else {
                val desiredVelocity = sqrt(lastState.v * lastState.v + 2 * maxAccel * dx)
                if (desiredVelocity <= maxVel) {
                    val state = MotionState(displacement, lastState.v, maxAccel)
                    forwardStates.add(Pair(state, dx))
                    state.afterDisplacement(dx)
                } else {
                    val accelDx =
                        (maxVel * maxVel - lastState.v * lastState.v) / (2 * maxAccel)
                    val accelState = MotionState(displacement, lastState.v, maxAccel)
                    val coastState = MotionState(displacement + accelDx, maxVel, 0.0)
                    forwardStates.add(Pair(accelState, accelDx))
                    forwardStates.add(Pair(coastState, dx - accelDx))
                    coastState.afterDisplacement(dx - accelDx)
                }
            }
        }

        return forwardStates
    }

    private fun intersection(state1: MotionState, state2: MotionState): Double {
        return (state1.v * state1.v - state2.v * state2.v) / (2 * state2.a - 2 * state1.a)
    }
}