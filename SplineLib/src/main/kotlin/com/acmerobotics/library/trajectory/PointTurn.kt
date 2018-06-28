package com.acmerobotics.library.trajectory

import com.acmerobotics.library.Angle
import com.acmerobotics.library.Pose2d
import com.acmerobotics.library.profile.MotionConstraints
import com.acmerobotics.library.profile.MotionProfile
import com.acmerobotics.library.profile.MotionProfileGenerator
import com.acmerobotics.library.profile.MotionState

// TODO: support CW turns
class PointTurn(val start: Pose2d, val endHeading: Double, val constraints: DriveConstraints): TrajectorySegment {
    val profile: MotionProfile
    val turnAngle: Double = if (endHeading >= start.heading) {
        endHeading - start.heading
    } else {
        2 * Math.PI - endHeading + start.heading
    }

    init {
        val start = MotionState(0.0, 0.0, 0.0)
        val goal = MotionState(turnAngle, 0.0, 0.0)
        profile = MotionProfileGenerator.generateMotionProfile(start, goal, object : MotionConstraints {
            override fun maximumVelocity(displacement: Double) = constraints.maximumAngularVelocity
            override fun maximumAcceleration(displacement: Double) = constraints.maximumAngularAcceleration
        })
    }

    override fun duration() = profile.duration()

    override fun get(time: Double) = Pose2d(start.x, start.y, Angle.norm(start.heading + profile[time].x))

    override fun velocity(time: Double) = Pose2d(0.0, 0.0, profile[time].v)

    override fun acceleration(time: Double) = Pose2d(0.0, 0.0, profile[time].a)

}