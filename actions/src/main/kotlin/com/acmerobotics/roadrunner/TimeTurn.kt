package com.acmerobotics.roadrunner

import kotlin.math.abs

data class TurnConstraints(
    val maxAngVel: Double,
    val minAngAccel: Double,
    val maxAngAccel: Double,
)

class TimeTurn(
    val beginPose: Pose2d,
    val angle: Double,
    val constraints: TurnConstraints,
) {
    // NOTE: I think the acceleration limit semantics here are what we want. Regardless of
    // whether the turn is to the left or to the right, max accel limit is applied ramping
    // up in speed and the min accel limit is applied ramping down.
    val profile = TimeProfile(
        constantProfile(
            abs(angle),
            0.0,
            constraints.maxAngVel,
            constraints.minAngAccel,
            constraints.maxAngAccel
        ).baseProfile
    )
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
