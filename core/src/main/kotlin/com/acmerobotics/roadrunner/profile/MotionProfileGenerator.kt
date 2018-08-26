package com.acmerobotics.roadrunner.profile

import com.acmerobotics.roadrunner.util.MathUtil
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Motion profile generator with arbitrary start and end motion states and either dynamic constraints or jerk limiting.
 */
object MotionProfileGenerator {
    /**
     * Generates a simple motion profile with constant [maximumVelocity], [maximumAcceleration], and [maximumJerk].
     *
     * @param start start motion state
     * @param goal goal motion state
     * @param maximumVelocity maximum velocity
     * @param maximumAcceleration maximum acceleration
     * @param maximumJerk maximum jerk (optional)
     */
    @JvmStatic
    @JvmOverloads
    fun generateSimpleMotionProfile(
        start: MotionState,
        goal: MotionState,
        maximumVelocity: Double,
        maximumAcceleration: Double,
        maximumJerk: Double = 0.0
    ): MotionProfile {
        if (abs(maximumJerk) < 1e-6) {
            return generateMotionProfile(
                    start,
                    goal,
                    SimpleMotionConstraints(maximumVelocity, maximumAcceleration),
                    1
            )
        }

        val noCoastProfile = generateNoCoastProfile(start, goal, maximumVelocity, maximumAcceleration, maximumJerk)
        val remainingDistance = goal.x - noCoastProfile.end().x

        if (remainingDistance >= 0.0) {
            // we just need to add a coast segment of appropriate duration
            val deltaT4 = remainingDistance / maximumVelocity
            val profile = MotionProfile(start)
            for (i in 0..2) {
                val segment = noCoastProfile.getSegment(i)
                profile.appendJerkControl(segment.start.j, segment.dt)
            }
            profile.appendJerkControl(0.0, deltaT4)
            for (i in 3..5) {
                val segment = noCoastProfile.getSegment(i)
                profile.appendJerkControl(segment.start.j, segment.dt)
            }
            return profile
        } else {
            // the profile never reaches maxV
            // thus, we need to compute the peak velocity (0 < peak vel < max vel)
            // we *could* construct a large polynomial expression (i.e., a nasty cubic) and solve it using Cardano's
            // method, some kind of inclusion method like modified Anderson-Bjorck-King, or a host of other methods
            // (see https://link.springer.com/content/pdf/bbm%3A978-3-642-05175-3%2F1.pdf for modified ABK)
            // instead, however, we conduct a binary search as it's sufficiently performant for this use case, requires
            // less code, and is overall significantly more comprehensible
            var upperBound = maximumVelocity
            var lowerBound = 0.0
            while (true) {
                val peakVel = (upperBound + lowerBound) / 2

                val profile = generateNoCoastProfile(start, goal, peakVel, maximumAcceleration, maximumJerk)
                val error = goal.x - profile.end().x

                if (abs(error) < 1e-10) {
                    return profile
                }

                if (error > 0.0) {
                    // we undershot so shift the lower bound up
                    lowerBound = peakVel
                } else {
                    // we overshot so shift the upper bound down
                    upperBound = peakVel
                }
            }
        }
    }

    private fun generateNoCoastProfile(
            start: MotionState,
            goal: MotionState,
            maximumVelocity: Double,
            maximumAcceleration: Double,
            maximumJerk: Double
    ): MotionProfile {
        val accelTimeDeltas = generateAccelProfileTimeDeltas(start, maximumVelocity, maximumAcceleration, maximumJerk)
        val decelTimeDeltas = generateAccelProfileTimeDeltas(goal, maximumVelocity, maximumAcceleration, maximumJerk).reversed()

        val noCoastProfile = MotionProfile(start)
        noCoastProfile.appendJerkControl(maximumJerk, accelTimeDeltas[0])
        noCoastProfile.appendJerkControl(0.0, accelTimeDeltas[1])
        noCoastProfile.appendJerkControl(-maximumJerk, accelTimeDeltas[2])
        noCoastProfile.appendJerkControl(-maximumJerk, decelTimeDeltas[0])
        noCoastProfile.appendJerkControl(0.0, decelTimeDeltas[1])
        noCoastProfile.appendJerkControl(maximumJerk, decelTimeDeltas[2])

        return noCoastProfile
    }

    private fun generateAccelProfileTimeDeltas(
            start: MotionState,
            maxVel: Double,
            maxAccel: Double,
            maxJerk: Double
    ): List<Double> {
        // compute the duration of the first and last acceleration profile segments
        val deltaT1 = (maxAccel - start.a) / maxJerk
        val deltaT3 = maxAccel / maxJerk

        // use these durations to find the changes in velocity
        val deltaV1 = start.a * deltaT1 + 0.5 * maxJerk * deltaT1 * deltaT1
        val deltaV3 = maxAccel * deltaT3 - 0.5 * maxJerk * deltaT3 * deltaT3
        val deltaV2 = maxVel - start.v - deltaV1 - deltaV3

        return if (deltaV2 < 0.0) {
            // there is no constant acceleration phase
            val roots = MathUtil.solveQuadratic(maxJerk, 2 * start.a, start.v - maxVel + start.a * start.a / (2 * maxJerk))
            // TODO: handle this degenerate case instead of throwing an exception
            val newDeltaT1 = roots.firstOrNull { it >= 0.0 } ?: throw RuntimeException("Invalid profile start state")
            val newDeltaT3 = newDeltaT1 + start.a / maxJerk

            listOf(newDeltaT1, 0.0, newDeltaT3)
        } else {
            // there is a constant acceleration phase
            val deltaT2 = deltaV2 / maxAccel

            listOf(deltaT1, deltaT2, deltaT3)
        }
    }

    /**
     * Generates a motion profile with dynamic maximum velocity and acceleration. Uses the algorithm described in section
     * 3.2 of [Sprunk2008.pdf](http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf).
     *
     * @param start start motion state
     * @param end end motion state
     * @param constraints motion constraints
     * @param resolution number of constraint samples
     */
    @JvmStatic
    fun generateMotionProfile(
        start: MotionState,
        goal: MotionState,
        constraints: MotionConstraints,
        resolution: Int = 250
    ): MotionProfile {
        if (goal.x < start.x) {
            return generateMotionProfile(
                goal,
                start,
                constraints,
                resolution
            ).reversed()
        }

        val length = goal.x - start.x
        val dx = length / resolution

        val forwardStates = forwardPass(
            MotionState(0.0, start.v, start.a),
            { constraints.maximumVelocity(start.x + it) },
            { constraints.maximumAcceleration(start.x + it) },
            resolution,
            dx
        ).map { (motionState, dx) -> Pair(
            MotionState(
                motionState.x + start.x,
                motionState.v,
                motionState.a
            ), dx) }
            .toMutableList()

        val backwardStates = forwardPass(
            MotionState(0.0, goal.v, goal.a),
            { constraints.maximumVelocity(goal.x - it) },
            { constraints.maximumAcceleration(goal.x - it) },
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
        while (i < forwardStates.size && i < backwardStates.size) {
            var (forwardStartState, forwardDx) = forwardStates[i]
            var (backwardStartState, backwardDx) = backwardStates[j]

            if (abs(forwardDx - backwardDx) > 1e-6) {
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
                    val intersection = intersection(
                        forwardStartState,
                        backwardStartState
                    )
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
                    val intersection = intersection(
                        forwardStartState,
                        backwardStartState
                    )
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
            val dt = if (abs(state.a) > 1e-6) {
                val discriminant = state.v * state.v + 2 * state.a * stateDx
                ((if (abs(discriminant) < 1e-6) 0.0 else sqrt(discriminant)) - state.v) / state.a
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