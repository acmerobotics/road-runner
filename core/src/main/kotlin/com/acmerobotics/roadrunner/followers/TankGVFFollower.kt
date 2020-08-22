package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.DisplacementState
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator.generateOnlineMotionProfile
import com.acmerobotics.roadrunner.profile.OnlineMotionProfile
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator.generateConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.GuidingVectorField
import com.acmerobotics.roadrunner.util.GuidingVectorField.ErrorMap
import com.acmerobotics.roadrunner.util.NanoClock
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.*

/**
 * State-of-the-art path follower based on the [GuidingVectorField].
 *
 * @param constraints robot motion constraints
 * @param admissibleError admissible/satisfactory pose error at the end of each move
 * @param kN normal vector weight (see [GuidingVectorField])
 * @param kTheta proportional heading error curvature gain
 * @param errorMap error map function (see [GuidingVectorField])
 * @param clock clock (used for timestamping updates and pid control)
 */
class TankGVFFollower @JvmOverloads constructor(
    private val constraints: DriveConstraints,
    admissibleError: Pose2d,
    private val kN: Double,
    private val kTheta: Double,
    private val errorMap: ErrorMap = ErrorMap.Linear(),
    clock: NanoClock = NanoClock.system()
) : PathFollower(admissibleError, clock) {
    private lateinit var gvf: GuidingVectorField
    private lateinit var profile: OnlineMotionProfile

    private var lastVel: Double = 0.0

    override var displacement: Double = Double.NaN
    override var lastError: Pose2d = Pose2d()

    override fun followPath(path: Path) {
        val profile = generateOnlineMotionProfile(
                DisplacementState(0.0),
                DisplacementState(0.0),
                path.length(),
                generateConstraints(path, constraints),
                clock
        )
        followPath(path, profile)
    }

    /**
     * Follows the given [path] and uses the provided [profile] for online motion profiling
     *
     * @param path the path to be followed
     * @param profile the user provided online profile to be used (see [generateOnlineMotionProfile] and
     * [generateConstraints])
     */
    fun followPath(path: Path, profile: OnlineMotionProfile) {
        gvf = GuidingVectorField(path, kN, errorMap)
        this.profile = profile
        lastVel = 0.0
        displacement = Double.NaN
        super.followPath(path)
    }

    override fun internalUpdate(currentPose: Pose2d, currentRobotVel: Pose2d?): DriveSignal {
        displacement = if (displacement.isNaN()) {
            path.project(currentPose.vec())
        } else {
            path.fastProject(currentPose.vec(), displacement)
        }

        val gvfResult = gvf.getExtended(currentPose.vec(), displacement)
        val t = path.reparam(displacement)
        val pathPose = path[displacement, t]
        val pathDeriv = path.deriv(displacement, t)
        val pathSecondDeriv = path.secondDeriv(displacement, t)

        val error = Kinematics.calculatePoseError(pathPose, currentPose)

        // Accounts for the path being reversed (when the heading is not facing the path derivative)
        val pathHeading = pathPose.headingVec()
        val tangentHeading = pathDeriv.vec()
        val isReversed = pathHeading dot tangentHeading < 0.0
        val direction = if (isReversed) -1.0 else 1.0
        val headingOffset = if (isReversed) PI else 0.0

        val desiredHeading = Angle.norm(gvfResult.vector.angle() + headingOffset)
        val headingError = Angle.normDelta(desiredHeading - currentPose.heading)

        val desiredHeadingDeriv = -gvfResult.deriv.rotated(PI / 2.0) dot gvfResult.vector
        val headingCorrection = kTheta * headingError
        val curvature = headingCorrection + desiredHeadingDeriv

        val dynamicDeriv = Pose2d(currentPose.headingVec(), curvature)
        val headingNormal = currentPose.headingVec().rotated(PI / 2)
        val dynamicSecondDeriv = Pose2d(gvfResult.deriv projectOnto headingNormal, pathSecondDeriv.heading)
        val constraints = constraints[displacement, currentPose, dynamicDeriv, dynamicSecondDeriv]

        // basic online motion profiling
        val profileState = profile[displacement, gvfResult.error, constraints]
        profile.update(profileState.v)

        val omega = curvature * profileState.v

        val robotVelocity = Pose2d(profileState.v * direction, 0.0, omega)
        val robotAcceleration = Pose2d(profileState.a * direction, 0.0, 0.0)

        // Small corrections at the end of the path are undesirable on tank drive, so the trajectory ends here
        if (displacement epsilonEquals path.length()) {
            executedFinalUpdate = true
        }

        lastVel = profileState.v

        lastError = error

        // TODO: GVF heading acceleration FF?
        return DriveSignal(robotVelocity, robotAcceleration)
    }
}
