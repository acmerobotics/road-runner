package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.DisplacementState
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator.generateOnlineMotionProfile
import com.acmerobotics.roadrunner.profile.OnlineMotionProfile
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator.generateConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.util.GuidingVectorField
import com.acmerobotics.roadrunner.util.GuidingVectorField.ErrorMap
import com.acmerobotics.roadrunner.util.NanoClock
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.*

/**
 * State-of-the-art path follower based on the [GuidingVectorField] with PID control for independent heading actuation.
 *
 * @param constraints robot motion constraints
 * @param admissibleError admissible/satisfactory pose error at the end of each move
 * @param kN normal vector weight (see [GuidingVectorField])
 * @param headingCoeffs independent heading controller PID gains
 * @param errorMap error map function (see [GuidingVectorField])
 * @param clock clock (used for timestamping updates and pid control)
 */
class HolonomicGVFFollower @JvmOverloads constructor(
    private val constraints: DriveConstraints,
    admissibleError: Pose2d,
    private val kN: Double,
    headingCoeffs: PIDCoefficients,
    private val errorMap: ErrorMap = ErrorMap.Linear(),
    clock: NanoClock = NanoClock.system()
) : PathFollower(admissibleError, clock) {
    private lateinit var gvf: GuidingVectorField
    private lateinit var profile: OnlineMotionProfile

    private val headingController = PIDFController(headingCoeffs, clock = clock).apply { setInputBounds(-PI, PI) }
    private var lastVel: Double = 0.0
    private var hasTerminated = false

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
        headingController.reset()
        lastVel = 0.0
        displacement = Double.NaN
        hasTerminated = false
        super.followPath(path)
    }

    @Suppress("ComplexMethod")
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

        val fieldError = pathPose.vec() - currentPose.vec()
        val error = Kinematics.calculatePoseError(pathPose, currentPose)

        // you can pass the error directly to PIDFController by setting setpoint = error and measurement = 0
        headingController.targetPosition = error.heading
        headingController.targetVelocity = pathDeriv.heading * lastVel * gvfResult.displacementDeriv

        // note: feedforward is processed at the wheel level
        val headingCorrection = headingController.update(0.0, currentRobotVel?.heading)

        // At the endpoint, the vector field continues in the same direction, so the pose error is used instead
        hasTerminated = hasTerminated || displacement epsilonEquals path.length()
        val gvfVector = if (hasTerminated) fieldError.unit() else gvfResult.vector
        val gvfDeriv = if (hasTerminated) Vector2d() else gvfResult.deriv

        var targetVel: Vector2d
        var targetAccel: Vector2d
        var omega: Double
        var alpha: Double
        var profileState = DisplacementState(lastVel)

        // Iteratively solve constraints (needed for the heading normalized correction which depends on velocity)
        var iters = 0
        do {
            targetVel = gvfVector * profileState.v
            targetAccel = gvfDeriv * profileState.v * profileState.v + gvfVector * profileState.a

            // Calculate derivs of displacement (along the path) with respect to time for calculating omega and alpha
            val displacementDeriv = gvfResult.displacementDeriv * profileState.v
            val pathOmega = pathDeriv.heading * displacementDeriv

            val denominator = 1.0 - ((currentPose - pathPose).vec() dot pathSecondDeriv.vec())
            val numerator1 =
                    displacementDeriv * (targetVel dot pathSecondDeriv.vec()) + (targetAccel dot pathDeriv.vec())
            val numerator2 = (targetVel dot pathDeriv.vec()) * (targetVel dot pathSecondDeriv.vec())
            val displacementSecondDeriv = numerator1 / denominator + numerator2 / (denominator * denominator)
            val pathAlpha = pathSecondDeriv.heading * displacementDeriv * displacementDeriv
                    + pathDeriv.heading * displacementSecondDeriv

            omega = (if (!hasTerminated) pathOmega else 0.0) + headingCorrection
            alpha = if (!hasTerminated) pathAlpha else 0.0

            val headingVelNormalized = if (omega epsilonEquals 0.0) 0.0 else omega / profileState.v
            val dynamicDeriv = Pose2d(gvfResult.vector, headingVelNormalized)
            // Path second deriv heading is not entirely correct here, but all constraints (at this time) do not use it
            val dynamicSecondDeriv = Pose2d(gvfResult.deriv, pathSecondDeriv.heading)
            val constraints = constraints[displacement, currentPose, dynamicDeriv, dynamicSecondDeriv]

            // basic online motion profiling
            val newProfileState = profile[displacement, gvfResult.error, constraints]

            if (newProfileState.v epsilonEquals profileState.v) {
                profileState = newProfileState
                break
            }

            profileState = newProfileState
        } while (iters++ < 20)

        profile.update(profileState.v)

        val fieldVelocity = Pose2d(targetVel, omega)
        val fieldAcceleration = Pose2d(targetAccel, alpha)
        val robotVelocity = Kinematics.fieldToRobotVelocity(currentPose, fieldVelocity)
        val robotAcceleration = Kinematics.fieldToRobotAcceleration(currentPose, fieldVelocity, fieldAcceleration)

        lastError = error
        lastVel = profileState.v

        return DriveSignal(robotVelocity, robotAcceleration)
    }
}
