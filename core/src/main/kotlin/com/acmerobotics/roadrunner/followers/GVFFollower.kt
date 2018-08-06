package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.drive.TankDrive
import com.acmerobotics.roadrunner.drive.TankKinematics
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.SimpleMotionConstraints
import kotlin.math.sign
import kotlin.math.sqrt

/**
 * State-of-the-art path follower based on the [GuidingVectorField].
 *
 * @param drive tank drive
 * @param constraints robot x velocity constraints
 * @param kN normal vector weight (see [GuidingVectorField])
 * @param kOmega proportional heading gain
 * @param kV feedforward velocity gain
 * @param kA feedforward acceleration gain (currently unused)
 * @param kStatic additive feedforward constant (used to overcome static friction)
 * @param errorMapFunc error map function (see [GuidingVectorField])
 */
class GVFFollower(
        private val drive: TankDrive,
        private val constraints: SimpleMotionConstraints,
        private val kN: Double,
        private val kOmega: Double,
        private val kV: Double,
        private val kA: Double,
        private val kStatic: Double,
        private val errorMapFunc: (Double) -> Double = { it }
) : PathFollower() {
    private lateinit var gvf: GuidingVectorField
    private var following: Boolean = false
    private var lastUpdateTimestamp: Double = 0.0
    private var lastVelocity: Double = 0.0

    override fun followPath(path: Path, startTimestamp: Double) {
        gvf = GuidingVectorField(path, kN, errorMapFunc)
        following = true
        lastUpdateTimestamp = startTimestamp
        lastVelocity = 0.0
        super.followPath(path, startTimestamp)
    }

    override fun isFollowing() = following

    override fun update(currentPose: Pose2d, currentTimestamp: Double) {
        if (!isFollowing()) {
            drive.setMotorPowers(0.0, 0.0)
            return
        }

        val gvfResult = gvf.getExtended(currentPose.x, currentPose.y)
        if (gvfResult.displacement > path.length()) {
            following = false
            drive.setMotorPowers(0.0, 0.0)
            return
        }

        val desiredHeading = Math.atan2(gvfResult.vector.y, gvfResult.vector.x)
        val headingError = currentPose.heading - desiredHeading

        // TODO: implement this or nah? ref eqs. (18), (23), and (24)
        val desiredOmega = 0.0
        val omega = desiredOmega - kOmega * headingError

        val dt = currentTimestamp - lastUpdateTimestamp
        val remainingDistance = currentPose.pos() distanceTo path.end().pos()
        val maxVelToStop = sqrt(2 * constraints.maximumAcceleration * remainingDistance)
        val maxVelFromLast = lastVelocity + constraints.maximumAcceleration * dt
        val velocity = minOf(maxVelFromLast, maxVelToStop, constraints.maximumVelocity)

        val wheelVelocities = TankKinematics.robotToWheelVelocities(Pose2d(velocity, 0.0, omega), drive.trackWidth)

        val motorPowers = wheelVelocities.map { it * kV + sign(it) * kStatic }
        drive.setMotorPowers(motorPowers[0], motorPowers[1])

        lastUpdateTimestamp = currentTimestamp
        lastVelocity = velocity
    }

}