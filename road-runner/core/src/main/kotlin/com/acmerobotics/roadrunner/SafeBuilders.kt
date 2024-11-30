package com.acmerobotics.roadrunner

class SafePathBuilder internal constructor(private val pathBuilder: PathBuilder) {
    constructor(beginPose: Pose2d, beginTangent: Rotation2d, eps: Double) :
        this(PathBuilder(beginPose, eps).setTangent(beginTangent))
    constructor(beginPose: Pose2d, beginTangent: Double, eps: Double) :
        this(beginPose, Rotation2d.exp(beginTangent), eps)

    fun lineToX(posX: Double) = TangentPathBuilder(pathBuilder.lineToX(posX))
    fun lineToXConstantHeading(posX: Double) = ConstantPathBuilder(pathBuilder.lineToXConstantHeading(posX))
    fun lineToXLinearHeading(posX: Double, heading: Rotation2d) =
        RestrictedPathBuilder(pathBuilder.lineToXLinearHeading(posX, heading))
    fun lineToXLinearHeading(posX: Double, heading: Double) = lineToXLinearHeading(posX, Rotation2d.exp(heading))
    fun lineToXSplineHeading(posX: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToXSplineHeading(posX, heading))
    fun lineToXSplineHeading(posX: Double, heading: Double) = lineToXSplineHeading(posX, Rotation2d.exp(heading))

    fun lineToY(posY: Double) = TangentPathBuilder(pathBuilder.lineToY(posY))
    fun lineToYConstantHeading(posY: Double) = ConstantPathBuilder(pathBuilder.lineToXConstantHeading(posY))
    fun lineToYLinearHeading(posY: Double, heading: Rotation2d) =
        RestrictedPathBuilder(pathBuilder.lineToYLinearHeading(posY, heading))
    fun lineToYLinearHeading(posY: Double, heading: Double) = lineToYLinearHeading(posY, Rotation2d.exp(heading))
    fun lineToYSplineHeading(posY: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToYSplineHeading(posY, heading))
    fun lineToYSplineHeading(posY: Double, heading: Double) = lineToYSplineHeading(posY, Rotation2d.exp(heading))

    fun splineTo(pos: Vector2d, tangent: Rotation2d) = TangentPathBuilder(pathBuilder.splineTo(pos, tangent))
    fun splineTo(pos: Vector2d, tangent: Double) = splineTo(pos, Rotation2d.exp(tangent))
    fun splineToConstantHeading(pos: Vector2d, tangent: Rotation2d) =
        ConstantPathBuilder(pathBuilder.splineToConstantHeading(pos, tangent))
    fun splineToConstantHeading(pos: Vector2d, tangent: Double) =
        splineToConstantHeading(pos, Rotation2d.exp(tangent))
    fun splineToLinearHeading(pose: Pose2d, tangent: Rotation2d) =
        RestrictedPathBuilder(pathBuilder.splineToLinearHeading(pose, tangent))
    fun splineToLinearHeading(pose: Pose2d, tangent: Double) = splineToLinearHeading(pose, Rotation2d.exp(tangent))
    fun splineToSplineHeading(pose: Pose2d, tangent: Rotation2d) =
        SafePathBuilder(pathBuilder.splineToSplineHeading(pose, tangent))
    fun splineToSplineHeading(pose: Pose2d, tangent: Double) = splineToSplineHeading(pose, Rotation2d.exp(tangent))

    fun build() = pathBuilder.build()
}

class TangentPathBuilder internal constructor(private val pathBuilder: PathBuilder) {
    fun lineToX(posX: Double) = TangentPathBuilder(pathBuilder.lineToX(posX))
    fun lineToXSplineHeading(posX: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToXSplineHeading(posX, heading))
    fun lineToXSplineHeading(posX: Double, heading: Double) = lineToXSplineHeading(posX, Rotation2d.exp(heading))

    fun lineToY(posY: Double) = TangentPathBuilder(pathBuilder.lineToY(posY))
    fun lineToYSplineHeading(posY: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToYSplineHeading(posY, heading))
    fun lineToYSplineHeading(posY: Double, heading: Double) = lineToYSplineHeading(posY, Rotation2d.exp(heading))

    fun splineTo(pos: Vector2d, tangent: Rotation2d) = TangentPathBuilder(pathBuilder.splineTo(pos, tangent))
    fun splineTo(pos: Vector2d, tangent: Double) = splineTo(pos, Rotation2d.exp(tangent))
    fun splineToSplineHeading(pose: Pose2d, tangent: Rotation2d) =
        SafePathBuilder(pathBuilder.splineToSplineHeading(pose, tangent))
    fun splineToSplineHeading(pose: Pose2d, tangent: Double) = splineToSplineHeading(pose, Rotation2d.exp(tangent))

    fun build() = pathBuilder.build()
}

class ConstantPathBuilder internal constructor(private val pathBuilder: PathBuilder) {
    fun lineToXConstantHeading(posX: Double) = ConstantPathBuilder(pathBuilder.lineToXConstantHeading(posX))
    fun lineToXSplineHeading(posX: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToXSplineHeading(posX, heading))
    fun lineToXSplineHeading(posX: Double, heading: Double) = lineToXSplineHeading(posX, Rotation2d.exp(heading))

    fun lineToYConstantHeading(posY: Double) = ConstantPathBuilder(pathBuilder.lineToXConstantHeading(posY))
    fun lineToYSplineHeading(posY: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToYSplineHeading(posY, heading))
    fun lineToYSplineHeading(posY: Double, heading: Double) = lineToYSplineHeading(posY, Rotation2d.exp(heading))

    fun splineToConstantHeading(pos: Vector2d, tangent: Rotation2d) =
        ConstantPathBuilder(pathBuilder.splineToConstantHeading(pos, tangent))
    fun splineToConstantHeading(pos: Vector2d, tangent: Double) =
        splineToConstantHeading(pos, Rotation2d.exp(tangent))
    fun splineToSplineHeading(pose: Pose2d, tangent: Rotation2d) =
        SafePathBuilder(pathBuilder.splineToSplineHeading(pose, tangent))
    fun splineToSplineHeading(pose: Pose2d, tangent: Double) = splineToSplineHeading(pose, Rotation2d.exp(tangent))

    fun build() = pathBuilder.build()
}

class RestrictedPathBuilder internal constructor(private val pathBuilder: PathBuilder) {
    fun lineToXSplineHeading(posX: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToXSplineHeading(posX, heading))
    fun lineToXSplineHeading(posX: Double, heading: Double) = lineToXSplineHeading(posX, Rotation2d.exp(heading))

    fun lineToYSplineHeading(posY: Double, heading: Rotation2d) =
        SafePathBuilder(pathBuilder.lineToYSplineHeading(posY, heading))
    fun lineToYSplineHeading(posY: Double, heading: Double) = lineToYSplineHeading(posY, Rotation2d.exp(heading))

    fun splineToSplineHeading(pose: Pose2d, tangent: Rotation2d) =
        SafePathBuilder(pathBuilder.splineToSplineHeading(pose, tangent))
    fun splineToSplineHeading(pose: Pose2d, tangent: Double) = splineToSplineHeading(pose, Rotation2d.exp(tangent))

    fun build() = pathBuilder.build()
}

/**
 * Wrapper for [PosePathSeqBuilder] that provides the same guarantees without throwing
 * [PosePathSeqBuilder.RotationContinuityException].
 *
 * For method-by-method documentation, see the identical methods on [PosePathSeqBuilder].
 */
class SafePosePathBuilder internal constructor(private val posePathSeqBuilder: PosePathSeqBuilder) {
    constructor(path: PositionPath<Arclength>, beginHeading: Rotation2d) :
        this(PosePathSeqBuilder(path, beginHeading))
    constructor(path: PositionPath<Arclength>, beginHeading: Double) :
        this(PosePathSeqBuilder(path, beginHeading))

    fun tangentUntil(disp: Double) =
        RestrictedPosePathBuilder(posePathSeqBuilder.tangentUntil(disp))
    fun constantUntil(disp: Double) =
        RestrictedPosePathBuilder(posePathSeqBuilder.constantUntil(disp))
    fun linearUntil(disp: Double, heading: Rotation2d) =
        RestrictedPosePathBuilder(posePathSeqBuilder.linearUntil(disp, heading))
    fun linearUntil(disp: Double, heading: Double) =
        RestrictedPosePathBuilder(posePathSeqBuilder.linearUntil(disp, heading))

    fun splineUntil(disp: Double, heading: Rotation2d) =
        SafePosePathBuilder(posePathSeqBuilder.splineUntil(disp, heading))
    fun splineUntil(disp: Double, heading: Double) =
        SafePosePathBuilder(posePathSeqBuilder.splineUntil(disp, heading))

    fun tangentUntilEnd() = posePathSeqBuilder.tangentUntilEnd()
    fun constantUntilEnd() = posePathSeqBuilder.constantUntilEnd()
    fun linearUntilEnd(heading: Rotation2d) = posePathSeqBuilder.linearUntilEnd(heading)
    fun linearUntilEnd(heading: Double) = posePathSeqBuilder.linearUntilEnd(heading)
    fun splineUntilEnd(heading: Rotation2d) = posePathSeqBuilder.splineUntilEnd(heading)
    fun splineUntilEnd(heading: Double) = posePathSeqBuilder.splineUntilEnd(heading)
}

/**
 * Do not instantiate directly. See [SafePosePathBuilder].
 */
class RestrictedPosePathBuilder internal constructor(private val posePathSeqBuilder: PosePathSeqBuilder) {
    fun splineUntil(disp: Double, heading: Rotation2d) =
        SafePosePathBuilder(posePathSeqBuilder.splineUntil(disp, heading))
    fun splineUntil(disp: Double, heading: Double) =
        SafePosePathBuilder(posePathSeqBuilder.splineUntil(disp, heading))

    fun splineUntilEnd(heading: Rotation2d) = posePathSeqBuilder.splineUntilEnd(heading)
    fun splineUntilEnd(heading: Double) = posePathSeqBuilder.splineUntilEnd(heading)
}

class SafeTrajectoryBuilder internal constructor(private val trajBuilder: TrajectoryBuilder) {
    @JvmOverloads
    constructor(
        params: TrajectoryBuilderParams,
        beginPose: Pose2d,
        beginTangent: Rotation2d,
        beginEndVel: Double,
        baseVelConstraint: VelConstraint,
        baseAccelConstraint: AccelConstraint,
        poseMap: PoseMap = IdentityPoseMap(),
    ) :
        this(
            TrajectoryBuilder(
                params,
                beginPose,
                beginEndVel, baseVelConstraint, baseAccelConstraint,
                poseMap,
            ).setTangent(beginTangent)
        )

    @JvmOverloads
    constructor(
        params: TrajectoryBuilderParams,
        beginPose: Pose2d,
        beginTangent: Double,
        beginEndVel: Double,
        baseVelConstraint: VelConstraint,
        baseAccelConstraint: AccelConstraint,
        poseMap: PoseMap = IdentityPoseMap(),
    ) :
        this(
            params,
            beginPose, Rotation2d.exp(beginTangent),
            beginEndVel, baseVelConstraint, baseAccelConstraint, poseMap
        )

    @JvmOverloads
    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TangentTrajectoryBuilder(trajBuilder.lineToX(posX, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToXConstantHeading(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = ConstantTrajectoryBuilder(
        trajBuilder.lineToXConstantHeading(
            posX, velConstraintOverride, accelConstraintOverride
        )
    )
    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        RestrictedTrajectoryBuilder(
            trajBuilder.lineToXLinearHeading(
                posX, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToXLinearHeading(posX, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToXSplineHeading(
                posX, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToXSplineHeading(posX, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TangentTrajectoryBuilder(trajBuilder.lineToY(posY, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToYConstantHeading(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = ConstantTrajectoryBuilder(
        trajBuilder.lineToXConstantHeading(
            posY, velConstraintOverride, accelConstraintOverride
        )
    )
    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        RestrictedTrajectoryBuilder(
            trajBuilder.lineToYLinearHeading(
                posY, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToYLinearHeading(posY, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToYSplineHeading(
                posY, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToYSplineHeading(posY, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TangentTrajectoryBuilder(trajBuilder.splineTo(pos, tangent, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = splineTo(pos, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        ConstantTrajectoryBuilder(
            trajBuilder.splineToConstantHeading(
                pos, tangent, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        splineToConstantHeading(pos, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        RestrictedTrajectoryBuilder(
            trajBuilder.splineToLinearHeading(
                pose, tangent, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = splineToLinearHeading(pose, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.splineToSplineHeading(
                pose, tangent, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = splineToSplineHeading(pose, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)

    fun build() = trajBuilder.build()
}

class TangentTrajectoryBuilder internal constructor(private val trajBuilder: TrajectoryBuilder) {
    @JvmOverloads
    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TangentTrajectoryBuilder(trajBuilder.lineToX(posX, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToXSplineHeading(
                posX, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToXSplineHeading(posX, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TangentTrajectoryBuilder(trajBuilder.lineToY(posY, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToYSplineHeading(
                posY, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToYSplineHeading(posY, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TangentTrajectoryBuilder(trajBuilder.splineTo(pos, tangent, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = splineTo(pos, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.splineToSplineHeading(
                pose, tangent, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = splineToSplineHeading(pose, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)

    fun build() = trajBuilder.build()
}

class ConstantTrajectoryBuilder internal constructor(private val trajBuilder: TrajectoryBuilder) {
    @JvmOverloads
    fun lineToXConstantHeading(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = ConstantTrajectoryBuilder(
        trajBuilder.lineToXConstantHeading(
            posX, velConstraintOverride, accelConstraintOverride
        )
    )
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToXSplineHeading(
                posX, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToXSplineHeading(posX, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToYConstantHeading(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = ConstantTrajectoryBuilder(
        trajBuilder.lineToXConstantHeading(
            posY, velConstraintOverride, accelConstraintOverride
        )
    )
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToYSplineHeading(
                posY, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToYSplineHeading(posY, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        ConstantTrajectoryBuilder(
            trajBuilder.splineToConstantHeading(
                pos, tangent, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        splineToConstantHeading(pos, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.splineToSplineHeading(
                pose, tangent, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = splineToSplineHeading(pose, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)

    fun build() = trajBuilder.build()
}

class RestrictedTrajectoryBuilder internal constructor(private val trajBuilder: TrajectoryBuilder) {
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToXSplineHeading(
                posX, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToXSplineHeading(posX, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.lineToYSplineHeading(
                posY, heading, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToYSplineHeading(posY, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        SafeTrajectoryBuilder(
            trajBuilder.splineToSplineHeading(
                pose, tangent, velConstraintOverride, accelConstraintOverride
            )
        )
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = splineToSplineHeading(pose, Rotation2d.exp(tangent), velConstraintOverride, accelConstraintOverride)

    fun build() = trajBuilder.build()
}
