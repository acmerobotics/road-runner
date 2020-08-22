package com.acmerobotics.roadrunner.profile

import com.acmerobotics.roadrunner.util.DoubleProgression
import com.acmerobotics.roadrunner.util.MathUtil.solveQuadratic
import com.acmerobotics.roadrunner.util.NanoClock
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.sqrt

/**
 * Motion profile generator with arbitrary start and end motion states and either dynamic constraints or jerk limiting.
 */
@Suppress("LargeClass", "ComplexMethod", "NestedBlockDepth")
object MotionProfileGenerator {

    /**
     * Generates a simple motion profile with constant [maxVel], [maxAccel], and [maxJerk]. If [maxJerk] is zero, an
     * acceleration-limited profile will be generated instead of a jerk-limited one. If constraints can't be obeyed,
     * there are two possible fallbacks: If [overshoot] is true, then two profiles will be concatenated (the first one
     * overshoots the goal and the second one reverses back to reach the goal). Otherwise, the highest order constraint
     * (e.g., max jerk for jerk-limited profiles) is repeatedly violated until the goal is achieved.
     *
     * @param start start motion state
     * @param goal goal motion state
     * @param maxVel maximum velocity
     * @param maxAccel maximum acceleration
     * @param maxJerk maximum jerk
     * @param overshoot if true overshoot otherwise violate constraints (see description above)
     */
    @Suppress("LongParameterList")
    @JvmStatic
    @JvmOverloads
    fun generateSimpleMotionProfile(
        start: MotionState,
        goal: MotionState,
        maxVel: Double,
        maxAccel: Double,
        maxJerk: Double = 0.0,
        overshoot: Boolean = false
    ): MotionProfile {
        // ensure the goal is always after the start; plan the flipped profile otherwise
        if (goal.x < start.x) {
            return generateSimpleMotionProfile(
                    start.flipped(),
                    goal.flipped(),
                    maxVel,
                    maxAccel,
                    maxJerk
            ).flipped()
        }

        if (maxJerk epsilonEquals 0.0) {
            // acceleration-limited profile (trapezoidal)
            val requiredAccel = (goal.v * goal.v - start.v * start.v) / (2 * (goal.x - start.x))

            val accelProfile = generateAccelProfile(start, maxVel, maxAccel)
            val decelProfile = generateAccelProfile(
                    MotionState(goal.x, goal.v, -goal.a, goal.j
                    ), maxVel, maxAccel, maxJerk)
                    .reversed()

            val noCoastProfile = accelProfile + decelProfile
            val remainingDistance = goal.x - noCoastProfile.end().x

            if (remainingDistance >= 0.0) {
                // normal 3-segment profile works
                val deltaT2 = remainingDistance / maxVel

                return MotionProfileBuilder(start)
                        .appendProfile(accelProfile)
                        .appendAccelerationControl(0.0, deltaT2)
                        .appendProfile(decelProfile)
                        .build()
            } else if (abs(requiredAccel) > maxAccel) {
                return if (overshoot) {
                    // TODO: is this most efficient? (do we care?)
                    noCoastProfile + generateSimpleMotionProfile(
                            noCoastProfile.end(),
                            goal,
                            maxVel,
                            maxAccel,
                            overshoot = true
                    )
                } else {
                    // single segment profile
                    val dt = (goal.v - start.v) / requiredAccel
                    MotionProfileBuilder(start)
                            .appendAccelerationControl(requiredAccel, dt)
                            .build()
                }
            } else if (start.v > maxVel && goal.v > maxVel) {
                // decel, accel
                val roots = solveQuadratic(-maxAccel, 2 * start.v,
                        (goal.v * goal.v - start.v * start.v) / (2 * maxAccel) - goal.x + start.x)
                val deltaT1 = roots.filter { it >= 0.0 }.min()!!
                val deltaT3 = abs(start.v - goal.v) / maxAccel + deltaT1

                return MotionProfileBuilder(start)
                        .appendAccelerationControl(-maxAccel, deltaT1)
                        .appendAccelerationControl(maxAccel, deltaT3)
                        .build()
            } else {
                // accel, decel
                val roots = solveQuadratic(maxAccel, 2 * start.v,
                        (start.v * start.v - goal.v * goal.v) / (2 * maxAccel) - goal.x + start.x)
                val deltaT1 = roots.filter { it >= 0.0 }.min()!!
                val deltaT3 = abs(start.v - goal.v) / maxAccel + deltaT1

                return MotionProfileBuilder(start)
                        .appendAccelerationControl(maxAccel, deltaT1)
                        .appendAccelerationControl(-maxAccel, deltaT3)
                        .build()
            }
        } else {
            // jerk-limited profile (S-curve)
            val accelerationProfile = generateAccelProfile(start, maxVel, maxAccel, maxJerk)
            // we leverage symmetry here; deceleration profiles are just reversed acceleration ones with the goal
            // acceleration flipped
            val decelerationProfile = generateAccelProfile(
                    MotionState(goal.x, goal.v, -goal.a, goal.j
                ), maxVel, maxAccel, maxJerk)
                    .reversed()

            val noCoastProfile = accelerationProfile + decelerationProfile
            val remainingDistance = goal.x - noCoastProfile.end().x

            if (remainingDistance >= 0.0) {
                // we just need to add a coast segment of appropriate duration
                val deltaT4 = remainingDistance / maxVel

                return MotionProfileBuilder(start)
                        .appendProfile(accelerationProfile)
                        .appendJerkControl(0.0, deltaT4)
                        .appendProfile(decelerationProfile)
                        .build()
            } else {
                // the profile never reaches maxV
                // thus, we need to compute the peak velocity (0 < peak vel < max vel)
                // we *could* construct a large polynomial expression (i.e., a nasty cubic) and solve it using Cardano's
                // method, some kind of inclusion method like modified Anderson-Bjorck-King, or a host of other methods
                // (see https://link.springer.com/content/pdf/bbm%3A978-3-642-05175-3%2F1.pdf for modified ABK)
                // instead, however, we conduct a binary search as it's sufficiently performant for this use case,
                // requires less code, and is overall significantly more comprehensible
                var upperBound = maxVel
                var lowerBound = 0.0
                var iterations = 0
                while (iterations < 1000) {
                    val peakVel = (upperBound + lowerBound) / 2

                    val searchAccelProfile = generateAccelProfile(start, peakVel, maxAccel, maxJerk)
                    val searchDecelProfile = generateAccelProfile(goal, peakVel, maxAccel, maxJerk)
                            .reversed()

                    val searchProfile = searchAccelProfile + searchDecelProfile

                    val error = goal.x - searchProfile.end().x

                    if (error epsilonEquals 0.0) {
                        return searchProfile
                    }

                    if (error > 0.0) {
                        // we undershot so shift the lower bound up
                        lowerBound = peakVel
                    } else {
                        // we overshot so shift the upper bound down
                        upperBound = peakVel
                    }

                    iterations++
                }

                // constraints are not satisfiable
                return if (overshoot) {
                    noCoastProfile + generateSimpleMotionProfile(
                            noCoastProfile.end(),
                            goal,
                            maxVel,
                            maxAccel,
                            maxJerk,
                            overshoot = true
                    )
                } else {
                    // violate max jerk first
                    generateSimpleMotionProfile(
                            start,
                            goal,
                            maxVel,
                            maxAccel,
                            overshoot = false
                    )
                }
            }
        }
    }

    private fun generateAccelProfile(
        start: MotionState,
        maxVel: Double,
        maxAccel: Double,
        maxJerk: Double = 0.0
    ): MotionProfile =
        if (maxJerk epsilonEquals 0.0) {
            // acceleration-limited
            val deltaT1 = abs(start.v - maxVel) / maxAccel
            val builder = MotionProfileBuilder(start)
            if (start.v > maxVel) {
                // we need to decelerate
                builder.appendAccelerationControl(-maxAccel, deltaT1)
            } else {
                builder.appendAccelerationControl(maxAccel, deltaT1)
            }
            builder.build()
        } else {
            // jerk-limited
            // compute the duration and velocity of the first segment
            val (deltaT1, deltaV1) = if (start.a > maxAccel) {
                // slow down and see where we are
                val deltaT1 = (start.a - maxAccel) / maxJerk
                val deltaV1 = start.a * deltaT1 - 0.5 * maxJerk * deltaT1 * deltaT1
                Pair(deltaT1, deltaV1)
            } else {
                // otherwise accelerate
                val deltaT1 = (maxAccel - start.a) / maxJerk
                val deltaV1 = start.a * deltaT1 + 0.5 * maxJerk * deltaT1 * deltaT1
                Pair(deltaT1, deltaV1)
            }

            // compute the duration and velocity of the third segment
            val deltaT3 = maxAccel / maxJerk
            val deltaV3 = maxAccel * deltaT3 - 0.5 * maxJerk * deltaT3 * deltaT3

            // compute the velocity change required in the second segment
            val deltaV2 = maxVel - start.v - deltaV1 - deltaV3

            if (deltaV2 < 0.0) {
                // there is no constant acceleration phase
                // the second case checks if we're going to exceed max vel
                if (start.a > maxAccel || (start.v - maxVel) > (start.a * start.a) / (2 * maxJerk)) {
                    // problem: we need to cut down on our acceleration but we can't cut our initial decel
                    // solution: we'll lengthen our initial decel to -max accel and similarly with our final accel
                    // if this results in an over correction, decel instead to a good accel
                    val newDeltaT1 = (start.a + maxAccel) / maxJerk
                    val newDeltaV1 = start.a * newDeltaT1 - 0.5 * maxJerk * newDeltaT1 * newDeltaT1

                    val newDeltaV2 = maxVel - start.v - newDeltaV1 + deltaV3

                    if (newDeltaV2 > 0.0) {
                        // we decelerated too much
                        val roots = solveQuadratic(-maxJerk, 2 * start.a,
                                start.v - maxVel - start.a * start.a / (2 * maxJerk))
                        val finalDeltaT1 = roots.filter { it >= 0.0 }.min()!!
                        val finalDeltaT3 = finalDeltaT1 - start.a / maxJerk

                        MotionProfileBuilder(start)
                                .appendJerkControl(-maxJerk, finalDeltaT1)
                                .appendJerkControl(maxJerk, finalDeltaT3)
                                .build()
                    } else {
                        // we're almost good
                        val newDeltaT2 = newDeltaV2 / -maxAccel

                        MotionProfileBuilder(start)
                                .appendJerkControl(-maxJerk, newDeltaT1)
                                .appendJerkControl(0.0, newDeltaT2)
                                .appendJerkControl(maxJerk, deltaT3)
                                .build()
                    }
                } else {
                    // cut out the constant accel phase and find a shorter delta t1 and delta t3
                    val roots = solveQuadratic(maxJerk, 2 * start.a,
                            start.v - maxVel + start.a * start.a / (2 * maxJerk))
                    val newDeltaT1 = roots.filter { it >= 0.0 }.min()!!
                    val newDeltaT3 = newDeltaT1 + start.a / maxJerk

                    MotionProfileBuilder(start)
                            .appendJerkControl(maxJerk, newDeltaT1)
                            .appendJerkControl(-maxJerk, newDeltaT3)
                            .build()
                }
            } else {
                // there is a constant acceleration phase
                val deltaT2 = deltaV2 / maxAccel

                val builder = MotionProfileBuilder(start)
                if (start.a > maxAccel) {
                    builder.appendJerkControl(-maxJerk, deltaT1)
                } else {
                    builder.appendJerkControl(maxJerk, deltaT1)
                }
                builder.appendJerkControl(0.0, deltaT2)
                        .appendJerkControl(-maxJerk, deltaT3)
                        .build()
            }
        }

    /**
     * Generates a motion profile with dynamic maximum velocity and acceleration. See [generateDisplacementProfile] for
     * more details.
     *
     * @param start start motion state
     * @param goal goal motion state
     * @param constraints motion constraints
     * @param resolution separation between constraint samples
     */
    @JvmStatic
    @JvmOverloads
    fun generateMotionProfile(
        start: MotionState,
        goal: MotionState,
        constraints: MotionConstraints,
        resolution: Double = 0.25
    ): MotionProfile {
        if (goal.x < start.x) {
            return generateMotionProfile(
                    start.flipped(),
                    goal.flipped(),
                    object : MotionConstraints() {
                        override fun get(s: Double) = constraints[-s]
                        override fun get(s: DoubleProgression) = constraints[-s]
                    },
                    resolution
            ).flipped()
        }

        val displacementProfile = generateDisplacementProfile(
                DisplacementState(start.v, start.a, start.j),
                DisplacementState(goal.v, goal.a, goal.j),
                goal.x - start.x,
                object : MotionConstraints() {
                    override fun get(s: Double) = constraints[s + start.x]
                    override fun get(s: DoubleProgression) = constraints[s + start.x]
                },
                resolution
        )

        return convertProfile(displacementProfile, start.x)
    }

    /**
     * Converts a [DisplacementProfile] to a time-indexed [MotionProfile] starting at x=[start]
     *
     * @param displacementProfile the displacement profile to be converted
     * @param start the start x that is added onto the dx from the displacement profile
     */
    fun convertProfile(displacementProfile: DisplacementProfile, start: Double): MotionProfile {
        val finalSegments = displacementProfile.segments

        // turn the final states into actual time-parametrized motion segments
        var x = start
        val motionSegments = mutableListOf<MotionSegment>()
        for (segment in finalSegments) {
            val state = segment.start
            val dt = segment.duration()
            motionSegments.add(
                    MotionSegment(MotionState(x, state.v, state.a, state.j), dt)
            )

            x += segment.dx
        }

        return MotionProfile(motionSegments)
    }

    /**
     * Generates an [OnlineMotionProfile] based on [generateDisplacementProfile] and the provided params
     *
     * @param start start displacement state
     * @param goal goal displacement state
     * @param length the length of the profile in displacement units
     * @param maxVel maximum velocity constraint
     * @param maxAccel maximum acceleration constraint
     * @param clock clock used for dt calculation
     */
    @Suppress("LongParameterList")
    @JvmStatic
    @JvmOverloads
    fun generateSimpleOnlineMotionProfile(
        start: DisplacementState,
        goal: DisplacementState,
        length: Double,
        maxVel: Double,
        maxAccel: Double,
        clock: NanoClock = NanoClock.system()
    ) = OnlineMotionProfile(
            start,
            goal,
            length,
            object : MotionConstraints() {
                override fun get(s: Double) = SimpleMotionConstraints(maxVel, maxAccel)
                override fun get(s: DoubleProgression) = s.map { get(it) }
            },
            clock
    )

    /**
     * Generates an [OnlineMotionProfile] based on [generateDisplacementProfile] and the provided params
     *
     * @param start start displacement state
     * @param goal goal displacement state
     * @param length the length of the profile in displacement units
     * @param constraints motion constraints
     * @param clock clock used for dt calculation
     * @param resolution separation between constraint samples
     */
    @Suppress("LongParameterList")
    @JvmStatic
    @JvmOverloads
    fun generateOnlineMotionProfile(
        start: DisplacementState,
        goal: DisplacementState,
        length: Double,
        constraints: MotionConstraints,
        clock: NanoClock = NanoClock.system(),
        resolution: Double = 0.25
    ): OnlineMotionProfile {
        // ds is an adjusted resolution that fits nicely within length
        val samples = ceil(length / resolution).toInt()
        val s = DoubleProgression.fromClosedInterval(0.0, length, samples)

        val constraintsList = constraints[s]
        val lastState = DisplacementState(constraintsList.last().maxVel)

        // we start with last constraint rather than goal because the end ramp is already handled in the online profile
        val backwardProfile = forwardPass(
                lastState,
                s,
                constraintsList.reversed()
        ).reversed()

        return OnlineMotionProfile(start, goal, length, constraints, clock, backwardProfile)
    }

    /**
     * Generates a displacement profile with dynamic maximum velocity and acceleration. Uses the algorithm described in
     * section 3.2 of [Sprunk2008.pdf](http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf). Warning:
     * Profiles may be generated incorrectly if the endpoint velocity/acceleration values preclude the obedience of the
     * motion constraints. To protect against this, verify the continuity of the generated profile or keep the start and
     * goal velocities at 0. Additionally, undershooting (start negative velocity or start negative acceleration beyond
     * velocity) and overshooting have undefined behavior.
     *
     * @param start start displacement state
     * @param goal goal displacement state
     * @param length the length of the profile in displacement units
     * @param constraints motion constraints
     * @param resolution separation between constraint samples
     */
    @JvmStatic
    @JvmOverloads
    fun generateDisplacementProfile(
        start: DisplacementState,
        goal: DisplacementState,
        length: Double,
        constraints: MotionConstraints,
        resolution: Double = 0.25
    ): DisplacementProfile {
        // ds is an adjusted resolution that fits nicely within length
        val samples = ceil(length / resolution).toInt()
        val s = DoubleProgression.fromClosedInterval(0.0, length, samples)

        val constraintsList = constraints[s]

        // compute the forward states
        val forwardProfile = forwardPass(
                start,
                s,
                constraintsList
        )

        // compute the backward states
        val backwardProfile = forwardPass(
                goal,
                s,
                constraintsList.reversed()
        ).reversed()

        // merge the forward and backward states
        return mergeProfiles(forwardProfile, backwardProfile)
    }

    // merges the forward profiles and reverse profiles together into a single constraint-respecting profile
    // See figure 3.4 of [Sprunk2008.pdf](http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf)
    private fun mergeProfiles(
        forwardProfile: DisplacementProfile,
        backwardProfile: DisplacementProfile
    ): DisplacementProfile {
        val forwardSegments = forwardProfile.segments.toMutableList()
        val backwardSegments = backwardProfile.segments.toMutableList()
        val finalSegments = mutableListOf<DisplacementSegment>()

        var i = 0
        while (i < forwardSegments.size && i < backwardSegments.size) {
            // retrieve the start states and displacement deltas
            var forwardSegment = forwardSegments[i]
            var backwardSegment = backwardSegments[i]

            // if there's a discrepancy in the displacements, split the the longer chunk in two and add the second
            // to the corresponding list; this guarantees that segments are always aligned
            if (!(forwardSegment.dx epsilonEquals backwardSegment.dx)) {
                if (forwardSegment.dx > backwardSegment.dx) {
                    // forward longer
                    forwardSegments.add(
                            i + 1,
                            DisplacementSegment(
                                    forwardSegment[backwardSegment.dx], forwardSegment.dx - backwardSegment.dx
                            )
                    )
                    forwardSegment = DisplacementSegment(forwardSegment.start, backwardSegment.dx)
                } else {
                    // backward longer
                    backwardSegments.add(
                            i + 1,
                            DisplacementSegment(
                                    backwardSegment[forwardSegment.dx], backwardSegment.dx - forwardSegment.dx
                            )
                    )
                    backwardSegment = DisplacementSegment(backwardSegment.start, forwardSegment.dx)
                }
            }

            // compute the end states (after alignment)
            val (higherSegment, lowerSegment) =
                    if (forwardSegment.start.v <= backwardSegment.start.v) {
                        Pair(backwardSegment, forwardSegment)
                    } else {
                        Pair(forwardSegment, backwardSegment)
                    }

            if (lowerSegment.end().v <= higherSegment.end().v) {
                // lower start, lower end
                finalSegments.add(lowerSegment)
            } else {
                // higher start, lower end
                val intersection = intersection(
                        lowerSegment.start,
                        higherSegment.start
                )
                finalSegments.add(DisplacementSegment(lowerSegment.start, intersection))
                finalSegments.add(
                        DisplacementSegment(
                                higherSegment[intersection],
                                higherSegment.dx - intersection
                        )
                )
            }

            i++
        }

        return DisplacementProfile(finalSegments)
    }

    // execute a forward pass that consists of applying maximum acceleration starting at min(last velocity, max vel)
    // on a segment-by-segment basis.
    // See figure 3.4 of [Sprunk2008.pdf](http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf)
    private fun forwardPass(
        start: DisplacementState,
        displacements: DoubleProgression,
        constraints: List<SimpleMotionConstraints>
    ): DisplacementProfile {
        val forwardSegments = mutableListOf<DisplacementSegment>()

        val dx = displacements.step

        var lastState = start
        constraints
            .dropLast(1)
            .forEach { constraint ->
                // compute the segment constraints
                val maxVel = constraint.maxVel
                val maxAccel = constraint.maxAccel

                lastState = if (lastState.v >= maxVel) {
                    // the last velocity exceeds max vel so we just coast
                    val segment = DisplacementSegment(DisplacementState(maxVel), dx)
                    forwardSegments.add(segment)
                    segment.end()
                } else {
                    // compute the final velocity assuming max accel
                    val finalVel = sqrt(lastState.v * lastState.v + 2 * maxAccel * dx)
                    if (finalVel <= maxVel) {
                        // we're still under max vel so we're good
                        val segment = DisplacementSegment(DisplacementState(lastState.v, maxAccel), dx)
                        forwardSegments.add(segment)
                        segment.end()
                    } else {
                        // we went over max vel so now we split the segment
                        val accelDx = (maxVel * maxVel - lastState.v * lastState.v) / (2 * maxAccel)
                        val accelState = DisplacementState(lastState.v, maxAccel)
                        val coastState = DisplacementState(maxVel)
                        val accelSegment = DisplacementSegment(accelState, accelDx)
                        val coastSegment = DisplacementSegment(coastState, dx - accelDx)
                        forwardSegments.add(accelSegment)
                        forwardSegments.add(coastSegment)
                        coastSegment.end()
                    }
                }
            }

        return DisplacementProfile(forwardSegments)
    }

    private fun intersection(state1: DisplacementState, state2: DisplacementState): Double {
        return (state1.v * state1.v - state2.v * state2.v) / (2 * state2.a - 2 * state1.a)
    }
}
