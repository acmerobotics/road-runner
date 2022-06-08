package com.acmerobotics.roadrunner

//import com.acmerobotics.roadrunner.control.PIDCoefficients
//import com.acmerobotics.roadrunner.control.PIDFController
//import com.acmerobotics.roadrunner.drive.Drive
//import com.acmerobotics.roadrunner.drive.DriveSignal
//import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
//import com.acmerobotics.roadrunner.geometry.Pose2d
//import com.acmerobotics.roadrunner.geometry.Vector2d
//import com.acmerobotics.roadrunner.kinematics.Kinematics
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
//import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
//import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
//import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint
//import com.acmerobotics.roadrunner.util.NanoClock
//import kotlin.math.PI
//
//// TODO: label implicit arguments?
//
//// TODO: include all the builtin paths
//// TODO: include all the markers



//
//// TODO: should I replace trajectory sequences with basic fsms?
//// I guess you would lose duration, position for dashboard display
//// maybe there should be two layers



//// or is this calculus that can be eliminated?
//// good markers definitely help with this use case
//
//fun pidManualTime() {
//    // TODO: simulated clock?
//    val currTs = mutableListOf(0.0)
//    val cont = PIDFController(
//        PIDCoefficients(0.0, 0.0, 0.0),
//        clock = object : NanoClock() {
//            override fun seconds() = currTs[0]
//        })
//
//    cont.targetPosition = 0.0
//    while (true) {
//        val ts = 0.0
//        val pos = 0.0
//        currTs[0] = ts
//        val power = cont.update(pos)
//    }
//}
//
//fun complementaryLocalizer() {
//
//}
//
//// probably a bit extra
//fun monteCarloLocalizer() {
//
//}
//
//fun buildinTrajGui() {
//
//}
//
//fun slowTrajectory() {
//    val traj = TrajectoryBuilder(Pose2d(0.0, 0.0, 0.0), false,
//        TranslationalVelocityConstraint(20.0),
//        ProfileAccelerationConstraint(15.0))
//        .lineTo(Vector2d(10.0, 0.0))
//        .lineTo(Vector2d(20.0, 0.0),
//            TranslationalVelocityConstraint(10.0), null)
//        .build()
//}
//
//fun driveFollowing() {
//
//}
//
//fun pathContinuitySafety() {
//
//}
//
//// current TS impl is only magically exception safe

//fun persistentBuilders() {
//    val builder =
//}

//
//fun mirrorPaths() {
//
//}
//
//fun goingBackwards() {
//
//}
//
//fun customWaypointDerivMag() {
//
//}
//
//fun admissibleTrajFollowing() {
//    val follower = HolonomicPIDVAFollower(
//        PIDCoefficients(0.0, 0.0, 0.0),
//        PIDCoefficients(0.0, 0.0, 0.0),
//        PIDCoefficients(0.0, 0.0, 0.0),
//        admissibleError = Pose2d(5.0, 5.0, PI / 8),
//        timeout = 0.25
//    )
//}
//
//fun orbitMode() {
//
//}
//

//fun fieldCentric(drive: Drive, leftStick: Vector2d, rightStick: Vector2d) {
//    drive.setDrivePower(Pose2d(
//        leftStick.rotated(-drive.poseEstimate.heading),
//        rightStick.x
//    ))

//}

fun getWheelIncrements(): WheelIncrements {
    return WheelIncrements(
        0.0, 0.0, 0.0, 0.0  // TODO: real measurements
    )
}

fun setWheelVelocities(vels: WheelVelocities<Time>) {

}

val TRANS_GAIN = 10.0
val ROT_GAIN = 0.1

fun goToPoint(kinematics: MecanumKinematics, initialPoseEstimate: Transform2, targetPose: Transform2) {
    var poseEstimate = initialPoseEstimate
    while (true) {
        poseEstimate += kinematics.forward(getWheelIncrements())
        val error = localError(targetPose, poseEstimate)
        // TODO: one could write some sugar
        // inverse() could take a Twist2
        val command = Twist2Dual.constant<Time>(Twist2(
            error.transError * TRANS_GAIN,
            error.rotError * ROT_GAIN,
        ), 1)
        setWheelVelocities(kinematics.inverse(command))
    }
}

//fun profiledTurn() {
//
//}
//
//fun profiledTurn180() {
//
//}
//
//fun cancelFollowing(clock: NanoClock, drive: Drive) {
//    val beginTs = clock.seconds()
//    val follower = HolonomicPIDVAFollower(
//        PIDCoefficients(0.0, 0.0, 0.0),
//        PIDCoefficients(0.0, 0.0, 0.0),
//        PIDCoefficients(0.0, 0.0, 0.0)
//    )
//    while (clock.seconds() < beginTs + 0.5 && follower.isFollowing()) {
//        drive.updatePoseEstimate()
//        drive.setDriveSignal(follower.update(drive.poseEstimate))
//    }
//    drive.setDriveSignal(DriveSignal())
//}
//
//fun spliceTraj() {
//
//}
//
//fun slowRegion() {
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
//}
//
//fun odometry() {
//    var pose = Pose2d(0.0, 0.0, 0.0)
//    while (true) {
//        val delta = Pose2d() // TODO
//        pose = Kinematics.relativeOdometryUpdate(pose, delta)
//    }
//}
//
//fun arm() {
//    val kG = 0.0
//    val elevCont = PIDFController(
//        PIDCoefficients(0.0, 0.0, 0.0),
//        0.0, 0.0, 0.0,
//        { _, _ -> kG }
//    )
//
//    elevCont.targetPosition = 0.0
//    while (true) {
//        val elevPos = 0.0
//        val elevVel = 0.0
//        val elevPower = elevCont.update(elevPos, elevVel)
//    }
//}
//
//
