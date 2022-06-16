package com.acmerobotics.roadrunner

import kotlin.math.PI

// import com.acmerobotics.roadrunner.control.PIDCoefficients
// import com.acmerobotics.roadrunner.control.PIDFController
// import com.acmerobotics.roadrunner.drive.Drive
// import com.acmerobotics.roadrunner.drive.DriveSignal
// import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
// import com.acmerobotics.roadrunner.geometry.Pose2d
// import com.acmerobotics.roadrunner.geometry.Vector2d
// import com.acmerobotics.roadrunner.kinematics.Kinematics
// import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
// import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
// import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
// import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
// import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint
// import com.acmerobotics.roadrunner.util.NanoClock
// import kotlin.math.PI
//
// // TODO: label implicit arguments?
//
// // TODO: include all the builtin paths
// // TODO: include all the markers

// fun displacementMarkers() {

fun getDisp(): Double {
    return 0.0
}

fun isFollowing(): Boolean {
    return false
}

fun main() {
    val posPath = PositionPathBuilder(
        Position2(0.0, 0.0),
        Rotation2.exp(0.0)
    )
        .splineTo(
            Position2(15.0, 15.0),
            Rotation2.exp(PI),
        )
        .splineTo(
            Position2(5.0, 35.0),
            Rotation2.exp(PI / 3),
        )
        .build()

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
    val builder = PositionPathBuilder(
        Position2(0.0, 0.0),
        Rotation2.exp(0.0)
    )
        .splineTo(
            Position2(15.0, 15.0),
            Rotation2.exp(PI),
        )

    val posPath1 = builder
        .splineTo(
            Position2(5.0, 35.0),
            Rotation2.exp(PI / 3),
        )
        .build()

    val posPath2 = builder
        .splineTo(
            Position2(5.0, 25.0),
            Rotation2.exp(PI / 3),
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

fun setWheelPowers(powers: WheelVelocities<Time>) {
}

fun fieldCentric(kinematics: MecanumKinematics, poseEstimate: Transform2, leftStick: Vector2, rightStick: Vector2) {
    setWheelPowers(
        kinematics.inverse(
            Twist2Dual.constant(
                poseEstimate.inverse() * Twist2(leftStick, rightStick.x),
                1
            )
        )
    )
}

fun getWheelIncrements(): WheelIncrements {
    return WheelIncrements(
        0.0, 0.0, 0.0, 0.0 // TODO: real measurements
    )
}

fun setWheelVelocities(vels: WheelVelocities<Time>) {
}

val TRANS_GAIN = 10.0
val ROT_GAIN = 0.1

fun goToPoint(kinematics: MecanumKinematics, initialPoseEstimate: Transform2, targetPose: Transform2) {
    var poseEstimate = initialPoseEstimate
    // TODO: termination criterion
    while (true) {
        // TODO: forward() may need some calculus to handle velocity measurements
        //  (eeeeeek then we need a dualized WheelIncr)
        // here it would be nice as a termination criterion
        poseEstimate += kinematics.forward(getWheelIncrements())
        val error = localError(targetPose, poseEstimate)
        // TODO: one could write some sugar
        // inverse() could take a Twist2
        val command = Twist2Dual.constant<Time>(
            Twist2(
                error.transError * TRANS_GAIN,
                error.rotError * ROT_GAIN,
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
    initialPoseEstimate: Transform2,
    maxAngVel: Double,
    maxAbsAngAccel: Double,
    angle: Double
) {
    // TODO: constant constraint overload would be nice with the resolution
    val profile = TimeProfile(
        profile(
            angle, 0.0,
            { maxAngVel }, { Interval(-maxAbsAngAccel, maxAbsAngAccel) }, angle
        )
    )
    // TODO: termination criterion

    var poseEstimate = initialPoseEstimate
    while (true) {
        poseEstimate += kinematics.forward(getWheelIncrements())

        val targetTurn = profile[clock()]
        val targetRot = initialPoseEstimate.rotation + targetTurn[0]
        val angError = targetRot - poseEstimate.rotation

        setWheelVelocities(
            kinematics.inverse(
                Twist2Dual(
                    Vector2Dual.constant(Vector2(0.0, 0.0), 2),
                    Rotation2Dual.exp(targetTurn).velocity() + angError * ROT_GAIN
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
