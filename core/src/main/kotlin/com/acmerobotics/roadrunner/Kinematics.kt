package com.acmerobotics.roadrunner

import kotlin.math.abs

/**
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
 * @param[lateralMultiplier] factor that multiplies strafe velocity to compensate for slip; increase it to boost the
 * distance traveled in the strafe direction
 */
data class MecanumKinematics @JvmOverloads constructor(
    @JvmField
    val trackWidth: Double,
    @JvmField
    val lateralMultiplier: Double = 1.0
) {
    /**
     * @param[wheelbase] distance between wheels on the same side; see the diagram in [MecanumKinematics]
     */
    constructor(
        trackWidth: Double,
        wheelbase: Double,
        lateralMultiplier: Double = 1.0
    ) : this((trackWidth + wheelbase) / 2, lateralMultiplier)

    data class WheelIncrements<Param>(
        @JvmField
        val leftFront: DualNum<Param>,
        @JvmField
        val leftBack: DualNum<Param>,
        @JvmField
        val rightBack: DualNum<Param>,
        @JvmField
        val rightFront: DualNum<Param>,
    )

    fun <Param> forward(w: WheelIncrements<Param>) = Twist2dDual(
        Vector2dDual(
            (w.leftFront + w.leftBack + w.rightBack + w.rightFront) * 0.25,
            (-w.leftFront + w.leftBack - w.rightBack + w.rightFront) * (0.25 / lateralMultiplier),
        ),
        (-w.leftFront - w.leftBack + w.rightBack + w.rightFront) * (0.25 / trackWidth),
    )

    data class WheelVelocities<Param>(
        @JvmField
        val leftFront: DualNum<Param>,
        @JvmField
        val leftBack: DualNum<Param>,
        @JvmField
        val rightBack: DualNum<Param>,
        @JvmField
        val rightFront: DualNum<Param>,
    ) {
        fun all() = listOf(leftFront, leftBack, rightBack, rightFront)
    }

    fun <Param> inverse(t: PoseVelocity2dDual<Param>) = WheelVelocities(
        t.linearVel.x - t.linearVel.y * lateralMultiplier - t.angVel * trackWidth,
        t.linearVel.x + t.linearVel.y * lateralMultiplier - t.angVel * trackWidth,
        t.linearVel.x - t.linearVel.y * lateralMultiplier + t.angVel * trackWidth,
        t.linearVel.x + t.linearVel.y * lateralMultiplier + t.angVel * trackWidth,
    )

    inner class WheelVelConstraint(@JvmField val maxWheelVel: Double) : VelConstraint {
        override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): Double {
            val txRobotWorld = robotPose.value().inverse()
            val robotVelWorld = robotPose.velocity().value()
            val robotVelRobot = txRobotWorld * robotVelWorld
            return inverse(PoseVelocity2dDual.constant<Arclength>(robotVelRobot, 1))
                .all()
                .minOf { abs(maxWheelVel / it.value()) }
        }
    }
}

/**
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
 * @param[wheelbase] distance between wheels on the same side; see the diagram in [MecanumKinematics]
 */
data class SwerveKinematics @JvmOverloads constructor(
    @JvmField
    val trackWidth: Double,
    @JvmField
    val wheelBase: Double
) {
    constructor(
        trackWidth: Double
    ) : this(trackWidth, trackWidth)

    data class WheelIncrements<Param>(
        @JvmField
        val leftFront: Vector2dDual<Param>,
        @JvmField
        val leftBack: Vector2dDual<Param>,
        @JvmField
        val rightBack: Vector2dDual<Param>,
        @JvmField
        val rightFront: Vector2dDual<Param>,
    )

    fun <Param> forward(w: WheelIncrements<Param>) = Twist2dDual(
        Vector2dDual(
            (w.leftFront.x + w.leftBack.x + w.rightBack.x + w.rightFront.x) * 0.25,
            (w.leftFront.y + w.leftBack.y + w.rightBack.y + w.rightFront.y) * 0.25
        ),
        (
                (-w.leftFront.x - w.leftBack.x + w.rightBack.x + w.rightFront.x) * 0.5 * trackWidth +
                ( w.leftFront.y - w.leftBack.y - w.rightBack.y + w.rightFront.y) * 0.5 * wheelBase
        ) / (wheelBase * wheelBase + trackWidth * trackWidth)
    )

    data class WheelVelocities<Param>(
        @JvmField
        val leftFront: Vector2dDual<Param>,
        @JvmField
        val leftBack: Vector2dDual<Param>,
        @JvmField
        val rightBack: Vector2dDual<Param>,
        @JvmField
        val rightFront: Vector2dDual<Param>,
    ) {
        fun all() = listOf(leftFront, leftBack, rightBack, rightFront)
    }

    fun <Param> inverse(t: PoseVelocity2dDual<Param>) = WheelVelocities(
        Vector2dDual(t.linearVel.x - t.angVel * trackWidth * 0.5, t.linearVel.y + t.angVel * wheelBase * 0.5),
        Vector2dDual(t.linearVel.x - t.angVel * trackWidth * 0.5, t.linearVel.y - t.angVel * wheelBase * 0.5),
        Vector2dDual(t.linearVel.x + t.angVel * trackWidth * 0.5, t.linearVel.y - t.angVel * wheelBase * 0.5),
        Vector2dDual(t.linearVel.x + t.angVel * trackWidth * 0.5, t.linearVel.y + t.angVel * wheelBase * 0.5)
    )

    inner class WheelVelConstraint(@JvmField val maxWheelVel: Double) : VelConstraint {
        override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): Double {
            val txRobotWorld = robotPose.value().inverse()
            val robotVelWorld = robotPose.velocity().value()
            val robotVelRobot = txRobotWorld * robotVelWorld
            return inverse(PoseVelocity2dDual.constant<Arclength>(robotVelRobot, 1))
                .all()
                .minOf { abs(maxWheelVel / it.value().norm()) }
        }
    }
}

/**
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
 */
data class TankKinematics(@JvmField val trackWidth: Double) {
    data class WheelIncrements<Param>(
        @JvmField
        val left: DualNum<Param>,
        @JvmField
        val right: DualNum<Param>,
    )

    fun <Param> forward(w: WheelIncrements<Param>) = Twist2dDual(
        Vector2dDual(
            (w.left + w.right) * 0.5,
            DualNum.constant(0.0, w.left.size()),
        ),
        (-w.left + w.right) / trackWidth,
    )

    data class WheelVelocities<Param>(
        @JvmField
        val left: DualNum<Param>,
        @JvmField
        val right: DualNum<Param>,
    ) {
        fun all() = listOf(left, right)
    }

    fun <Param> inverse(t: PoseVelocity2dDual<Param>): WheelVelocities<Param> {
        require(t.linearVel.y.values().all { abs(it) < 1e-6 })

        return WheelVelocities(
            t.linearVel.x - t.angVel * 0.5 * trackWidth,
            t.linearVel.x + t.angVel * 0.5 * trackWidth,
        )
    }

    // TODO: can probably be made generic, though lack of associated types may pose a difficulty
    inner class WheelVelConstraint(@JvmField val maxWheelVel: Double) : VelConstraint {
        override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): Double {
            val txRobotWorld = robotPose.value().inverse()
            val robotVelWorld = robotPose.velocity().value()
            val robotVelRobot = txRobotWorld * robotVelWorld
            return inverse(PoseVelocity2dDual.constant<Arclength>(robotVelRobot, 1))
                .all()
                .minOf { abs(maxWheelVel / it.value()) }
        }
    }
}
