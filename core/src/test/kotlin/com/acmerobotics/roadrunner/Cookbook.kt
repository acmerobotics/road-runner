package com.acmerobotics.roadrunner

import kotlin.math.PI

// TODO: label implicit arguments?

// TODO: include all the builtin paths
// TODO: include all the markers

// fun displacementMarkers() {

fun getDisp(): Double {
    return 0.0
}

fun isFollowing(): Boolean {
    return false
}

fun main() {
    val posPath = PositionPathSeqBuilder(
        Vector2d(0.0, 0.0),
        Rotation2d.exp(0.0),
        1e-6,
    )
        .splineTo(
            Vector2d(15.0, 15.0),
            Rotation2d.exp(PI),
        )
        .splineTo(
            Vector2d(5.0, 35.0),
            Rotation2d.exp(PI / 3),
        )
        .build()
        .first()

    // TODO: wrap this in a disposable command?
    val marker = posPath.offsets[1]
    var markerExecuted = false
    while (isFollowing()) {
        if (getDisp() > marker && !markerExecuted) {
            // execute marker action
            markerExecuted = true
        }
    }
    if (!markerExecuted) {
        // execute marker action
    }
}

// // TODO: should I replace trajectory sequences with basic fsms?
// // I guess you would lose duration, position for dashboard display
// // maybe there should be two layers

// fun complementaryLocalizer() {
//
// }
//
// // probably a bit extra
// fun monteCarloLocalizer() {
//
// }
//
// fun builtinTrajGui() {
//
// }
//
// TODO: should constraints have a time builder?
// I suppose overrides can be built in a layer on top of that
// fun slowTrajectory() {
//    val traj = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), false,
//        TranslationalVelocityConstraint(20.0),
//        ProfileAccelerationConstraint(15.0))
//        .lineTo(Vector2d(10.0, 0.0))
//        .lineTo(Vector2d(20.0, 0.0),
//            TranslationalVelocityConstraint(10.0), null)
//        .build()
// }
//
// fun driveFollowing() {
//
// }
//
// fun pathContinuitySafety() {
//
// }
//

// current TS impl is only magically exception safe
fun persistentBuilders() {
    val builder = PositionPathSeqBuilder(
        Vector2d(0.0, 0.0),
        Rotation2d.exp(0.0),
        1e-6,
    )
        .splineTo(
            Vector2d(15.0, 15.0),
            Rotation2d.exp(PI),
        )

    val posPath1 = builder
        .splineTo(
            Vector2d(5.0, 35.0),
            Rotation2d.exp(PI / 3),
        )
        .build()

    val posPath2 = builder
        .splineTo(
            Vector2d(5.0, 25.0),
            Rotation2d.exp(PI / 3),
        )
        .build()
}

// TODO: mirroring can be done to the inputs or the outputs
// output might just win for being easiest
// fun mirrorPaths() {
//
// }

// fun goingBackwards() {
//
// }

// TODO: important?
// fun customWaypointDerivMag() {
//
// }

// fun admissibleTrajFollowing() {
//    val follower = HolonomicPIDVAFollower(
//        PIDCoefficients(0.0, 0.0, 0.0),
//        PIDCoefficients(0.0, 0.0, 0.0),
//        PIDCoefficients(0.0, 0.0, 0.0),
//        admissibleError = Pose2d(5.0, 5.0, PI / 8),
//        timeout = 0.25
//    )
// }

// fun orbitMode() {
//
// }

fun setWheelPowers(powers: MecanumKinematics.WheelVelocities<Time>) {
}

fun fieldCentric(kinematics: MecanumKinematics, poseEstimate: Pose2d, leftStick: Vector2d, rightStick: Vector2d) {
    setWheelPowers(
        kinematics.inverse(
            PoseVelocity2dDual.constant(
                poseEstimate.inverse() * PoseVelocity2d(leftStick, rightStick.x),
                1
            )
        )
    )
}

fun getWheelIncrements(): MecanumKinematics.WheelIncrements<Time> {
    return MecanumKinematics.WheelIncrements(
        DualNum(doubleArrayOf(0.0)),
        DualNum(doubleArrayOf(0.0)),
        DualNum(doubleArrayOf(0.0)),
        DualNum(doubleArrayOf(0.0)),
    )
}

fun setWheelVelocities(vels: MecanumKinematics.WheelVelocities<Time>) {
}

val TRANS_GAIN = 10.0
val ROT_GAIN = 0.1

fun goToPoint(kinematics: MecanumKinematics, initialPoseEstimate: Pose2d, targetPose: Pose2d) {
    var poseEstimate = initialPoseEstimate
    // TODO: termination criterion
    while (true) {
        // TODO: forward() may need some calculus to handle velocity measurements
        //  (eeeeeek then we need a dualized WheelIncr)
        // here it would be nice as a termination criterion
        poseEstimate += kinematics.forward(getWheelIncrements()).value()
        val error = targetPose.minusExp(poseEstimate)
        // TODO: one could write some sugar
        // inverse() could take a Twist2
        val command = PoseVelocity2dDual.constant<Time>(
            PoseVelocity2d(
                error.position * TRANS_GAIN,
                error.heading.log() * ROT_GAIN,
            ),
            1
        )
        // TODO: this leaves out feedforward
        setWheelVelocities(kinematics.inverse(command))
    }
}

fun clock(): Double {
    return 0.0
}

fun turnWithProfile(
    kinematics: MecanumKinematics,
    initialPoseEstimate: Pose2d,
    maxAngVel: Double,
    maxAbsAngAccel: Double,
    angle: Double
) {
    val profile = TimeProfile(
        constantProfile(angle, 0.0, maxAngVel, -maxAbsAngAccel, maxAbsAngAccel).baseProfile
    )
    // TODO: termination criterion

    var poseEstimate = initialPoseEstimate
    while (true) {
        poseEstimate += kinematics.forward(getWheelIncrements()).value()

        val targetTurn = profile[clock()]
        val targetRot = initialPoseEstimate.heading + targetTurn[0]
        val angError = targetRot - poseEstimate.heading

        setWheelVelocities(
            kinematics.inverse(
                PoseVelocity2dDual(
                    Vector2dDual.constant(Vector2d(0.0, 0.0), 2),
                    Rotation2dDual.exp(targetTurn).velocity() + angError * ROT_GAIN
                )
            )
        )
    }
}

// fun spliceTraj() {
//
// }

// fun slowRegion() {
//    class RectangleMaskConstraint(
//        val minX: Double, val minY: Double,
//        val maxX: Double, val maxY: Double,
//        val c: TrajectoryVelocityConstraint
//    ) : TrajectoryVelocityConstraint {
//        override fun get(s: Double, pose: Pose2d, deriv: Pose2d, baseRobotVel: Pose2d) =
//            if (pose.x in minX..maxX && pose.y in minY..maxY) {
//                c[s, pose, deriv, baseRobotVel]
//            } else {
//                Double.POSITIVE_INFINITY
//            }
//    }
//
//    val traj = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), false,
//        MinVelocityConstraint(listOf(
//            TranslationalVelocityConstraint(50.0),
//            RectangleMaskConstraint(20.0, 20.0, 40.0, 40.0,
//                TranslationalVelocityConstraint(10.0))
//        )),
//        ProfileAccelerationConstraint(50.0)
//    )
//        .splineTo(Vector2d(40.0, 60.0), PI / 2)
//        .build()
// }
