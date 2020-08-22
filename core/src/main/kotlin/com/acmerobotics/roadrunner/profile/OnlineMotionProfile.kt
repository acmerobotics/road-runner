package com.acmerobotics.roadrunner.profile

import com.acmerobotics.roadrunner.util.NanoClock
import kotlin.math.*

/**
 * An online trapezoidal motion profile that is based on internal state and a backwards pass in a [DisplacementProfile].
 *
 * @param start start displacement state
 * @param goal goal displacement state
 * @param length the length of the profile in displacement units
 * @param baseConstraints motion constraints used for online mp unless an override is specified in [get]
 * @param clock clock used for dt calculation
 * @param backwardsPassProfile a backwards pass (usually with the same constraints as the forwards pass) used to ensure
 * decel consistency with dynamic constraints
 */
class OnlineMotionProfile(
    start: DisplacementState,
    private val goal: DisplacementState,
    private val length: Double,
    private val baseConstraints: MotionConstraints,
    private val clock: NanoClock = NanoClock.system(),
    private val backwardsPassProfile: DisplacementProfile? = null
) {
    private var lastVelocity: Double = start.v
    private var lastUpdateTimestamp: Double = clock.seconds()

    /**
     * Gets the velocity and numeric acceleration state based on the online mp forward pass and the
     * [DisplacementProfile] reverse pass.
     * Note: Calling [update] is required to update the internal state after a velocity has been selected.
     * @param displacement The displacement along the profile from `[0, length]`
     * @param error An optional error parameter that is factored into the stopping distance calculation for
     * multi-dimensional systems
     * @param constraints An optional parameter to override the constraints which can be useful for dynamic
     * situations
     */
    @JvmOverloads
    operator fun get(
        displacement: Double,
        error: Double = 0.0,
        constraints: SimpleMotionConstraints = baseConstraints[displacement]
    ): DisplacementState {
        val timestamp = clock.seconds()
        val dt = timestamp - lastUpdateTimestamp
        val remainingDisplacement = max(abs(length - displacement), abs(error))
        val maxVelToStop = sqrt(2.0 * constraints.maxAccel * remainingDisplacement) + goal.v
        val maxVelFromLast = lastVelocity + constraints.maxAccel * dt
        val maxVelForward = backwardsPassProfile?.get(displacement)?.v
        val maxOnlineVel = minOf(maxVelFromLast, maxVelToStop, constraints.maxVel)
        val velocity = maxVelForward?.let { min(it, maxOnlineVel) } ?: maxOnlineVel
        val acceleration = (velocity - lastVelocity) / dt

        return DisplacementState(velocity, acceleration)
    }

    /**
     * Updates the internal state (timestamp and previous velocity) after a given velocity has been selected
     * @param velocity The velocity set/used
     * @param timestamp An optional parameter to override the current timestamp which represents the time this velocity
     * was set
     */
    fun update(velocity: Double, timestamp: Double = clock.seconds()) {
        lastVelocity = velocity
        lastUpdateTimestamp = timestamp
    }
}
