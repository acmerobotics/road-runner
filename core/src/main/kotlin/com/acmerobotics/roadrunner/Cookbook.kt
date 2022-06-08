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
//
//
////is there a way
////to offset a trajectory by displacement
////like "i want to start at 2 inches displacement"
////or something like that
////im trying to do some like cancellation stuff with trajectories for cycling the warehouse
////but doing it by time is kinda iffy because my accel values for going into the warehouse vs going out are different
//
//// Is this an argument for trajectory interfaces?
//// Anything an API class accepts should probably be an interface to remain flexible
//// There's only so much direct flow control that can be accomplished
//
//// Like which of the following is better?
//// class Trajectory(val mp: Profile, val p: Path) { fun get(t: Double): DualNum<Pose2d, Vector2d, Time> { ... } }
//// trajGet(val s: DualNum<Disp, Double, Time>, val x: DualNum<Pose2d, Vector2d, Disp>): DualNum<Pose2d, Vector2d, Time> { ... }
//
//// same with trajectory followers... should they have memory?
//// convergence to the target is a separate question
//
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
//
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
//
//fun goToPoint() {
//
//}
//
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
