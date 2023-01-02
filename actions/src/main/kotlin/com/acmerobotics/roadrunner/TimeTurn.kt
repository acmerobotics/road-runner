package com.acmerobotics.roadrunner

import kotlin.math.abs

data class TurnConstraints(
    @JvmField
    val maxAngVel: Double,
    @JvmField
    val minAngAccel: Double,
    @JvmField
    val maxAngAccel: Double,
)

class TimeTurn(
    @JvmField
    val beginPose: Pose2d,
    @JvmField
    val angle: Double,
    @JvmField
    val constraints: TurnConstraints,
) {
    // NOTE: I think the acceleration limit semantics here are what we want. Regardless of
    // whether the turn is to the left or to the right, max accel limit is applied ramping
    // up in speed and the min accel limit is applied ramping down.
    @JvmField
    val profile = TimeProfile(
        constantProfile(
            abs(angle),
            0.0,
            constraints.maxAngVel,
            constraints.minAngAccel,
            constraints.maxAngAccel
        ).baseProfile
    )

    @JvmField
    val duration = profile.duration

    @JvmField
    val reversed = angle < 0

    operator fun get(t: Double): Pose2dDual<Time> {
        val x = profile[t]
        return Pose2dDual(
            Vector2dDual.constant(beginPose.trans, x.size()),
            Rotation2dDual.exp(
                if (reversed) {
                    x.unaryMinus()
                } else {
                    x
                }
            ) * beginPose.rot
        )
    }
}
