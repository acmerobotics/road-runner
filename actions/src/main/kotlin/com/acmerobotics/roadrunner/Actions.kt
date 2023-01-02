@file:JvmName("Actions")

package com.acmerobotics.roadrunner

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket

// NOTE: Actions need not be immutable. Most will return this from run()
interface Action {
    fun run(p: TelemetryPacket): Action?
    fun preview(c: Canvas)
}

data class ParallelAction(val actions: List<Action>) : Action {
    constructor(vararg actions: Action) : this(actions.asList())

    override fun run(p: TelemetryPacket) =
        actions.mapNotNull { it.run(p) }.let { actions ->
            if (actions.isEmpty()) {
                null
            } else {
                ParallelAction(actions)
            }
        }

    override fun preview(c: Canvas) {
        for (a in actions) {
            a.preview(c)
        }
    }
}

data class SequentialAction(val actions: List<Action>) : Action {
    constructor(vararg actions: Action) : this(actions.asList())

    override fun run(p: TelemetryPacket): Action? =
        if (actions.isEmpty()) {
            null
        } else {
            val a = actions.first().run(p)
            if (a == null) {
                SequentialAction(actions.drop(1)).run(p)
            } else {
                SequentialAction(listOf(a) + actions.drop(1))
            }
        }

    override fun preview(c: Canvas) {
        for (a in actions) {
            a.preview(c)
        }
    }
}

fun now() = System.nanoTime() * 1e-9

data class SleepAction(val dt: Double) : Action {
    private var beginTs = -1.0

    override fun run(p: TelemetryPacket): Action? {
        val t = if (beginTs < 0) {
            beginTs = now()
            0.0
        } else {
            now() - beginTs
        }

        return if (t < dt) {
            this
        } else {
            null
        }
    }

    override fun preview(c: Canvas) {}
}

private fun seqCons(hd: Action, tl: Action): Action =
    if (tl is SequentialAction) {
        SequentialAction(listOf(hd) + tl.actions)
    } else {
        SequentialAction(hd, tl)
    }

sealed class MarkerFactory(
    val segmentIndex: Int,
) {
    abstract fun make(t: TimeTrajectory, segmentDisp: Double): Action
}

class TimeMarkerFactory(segmentIndex: Int, val dt: Double, val a: Action) : MarkerFactory(segmentIndex) {
    override fun make(t: TimeTrajectory, segmentDisp: Double) =
        seqCons(SleepAction(t.profile.inverse(segmentDisp) + dt), a)
}

class DispMarkerFactory(segmentIndex: Int, val ds: Double, val a: Action) : MarkerFactory(segmentIndex) {
    override fun make(t: TimeTrajectory, segmentDisp: Double) =
        seqCons(SleepAction(t.profile.inverse(segmentDisp + ds)), a)
}

interface TrajectoryActionFactory {
    fun fromTurn(t: TimeTurn): Action
    fun fromTrajectory(t: TimeTrajectory): Action
}

class TrajectoryActionBuilder private constructor(
    // constants
    val factory: TrajectoryActionFactory,
    val eps: Double,
    val baseTurnConstraints: TurnConstraints,
    val baseVelConstraint: VelConstraint,
    val baseAccelConstraint: AccelConstraint,
    val resolution: Double,
    val poseMap: TrajectoryBuilder.PoseMap,
    // vary throughout
    private val tb: TrajectoryBuilder,
    private val n: Int,
    private val lastPose: Pose2d,
    private val lastTangent: Rotation2d,
    private val ms: List<MarkerFactory>,
    private val cont: (Action) -> Action,
) {
    @JvmOverloads
    constructor(
        factory: TrajectoryActionFactory,
        beginPose: Pose2d,
        eps: Double,
        baseTurnConstraints: TurnConstraints,
        baseVelConstraint: VelConstraint,
        baseAccelConstraint: AccelConstraint,
        resolution: Double,
        poseMap: TrajectoryBuilder.PoseMap = TrajectoryBuilder.PoseMap { it },
    ) :
        this(
            factory,
            eps,
            baseTurnConstraints,
            baseVelConstraint,
            baseAccelConstraint,
            resolution,
            poseMap,
            TrajectoryBuilder(
                beginPose, eps,
                beginEndVel = 0.0, baseVelConstraint, baseAccelConstraint, resolution,
                poseMap,
            ),
            0,
            beginPose,
            beginPose.rot,
            emptyList(),
            { it },
        )

    private constructor(
        ab: TrajectoryActionBuilder,
        tb: TrajectoryBuilder,
        n: Int,
        lastPose: Pose2d,
        lastTangent: Rotation2d,
        ms: List<MarkerFactory>,
        cont: (Action) -> Action,
    ) :
        this(
            ab.factory,
            ab.eps,
            ab.baseTurnConstraints,
            ab.baseVelConstraint,
            ab.baseAccelConstraint,
            ab.resolution,
            ab.poseMap,
            tb,
            n,
            lastPose,
            lastTangent,
            ms,
            cont
        )

    fun endTrajectory() =
        if (n == 0) {
            require(ms.isEmpty())

            this
        } else {
            val ts = tb.build()
            val end = ts.last().path.end(2)
            val endPose = end.value()
            val endTangent = end.velocity().value().transVel.angleCast()
            TrajectoryActionBuilder(
                this,
                TrajectoryBuilder(
                    endPose,
                    eps,
                    beginEndVel = 0.0,
                    baseVelConstraint,
                    baseAccelConstraint,
                    resolution,
                    poseMap
                ),
                0,
                endPose,
                endTangent,
                emptyList()
            ) { tail ->
                val (aNew, msRem) = ts.zip(ts.scan(0) { acc, t -> acc + t.offsets.size }).foldRight(
                    Pair(tail, ms)
                ) { (traj, offset), (acc, ms) ->
                    val timeTraj = TimeTrajectory(traj)
                    val actions = mutableListOf(seqCons(factory.fromTrajectory(timeTraj), acc))
                    val msRem = mutableListOf<MarkerFactory>()
                    for (m in ms) {
                        val i = m.segmentIndex - offset
                        if (i >= 0) {
                            actions.add(m.make(timeTraj, traj.offsets[i]))
                        } else {
                            msRem.add(m)
                        }
                    }

                    if (actions.size == 1) {
                        Pair(actions.first(), msRem)
                    } else {
                        Pair(ParallelAction(actions), msRem)
                    }
                }

                require(msRem.isEmpty())

                cont(aNew)
            }
        }

    fun stopAndAdd(a: Action): TrajectoryActionBuilder {
        val b = endTrajectory()
        return TrajectoryActionBuilder(b, b.tb, b.n, b.lastPose, b.lastTangent, b.ms) { tail ->
            b.cont(seqCons(a, tail))
        }
    }

    fun waitSeconds(t: Double) = stopAndAdd(SleepAction(t))

    fun afterDisp(ds: Double, a: Action) =
        TrajectoryActionBuilder(
            this, tb, n, lastPose, lastTangent,
            ms + listOf(DispMarkerFactory(n, ds, a)), cont
        )

    fun afterTime(dt: Double, a: Action) =
        if (n == 0) {
            TrajectoryActionBuilder(this, tb, 0, lastPose, lastTangent, emptyList()) { tail ->
                cont(ParallelAction(tail, seqCons(SleepAction(dt), a)))
            }
        } else {
            TrajectoryActionBuilder(
                this, tb, n, lastPose, lastTangent,
                ms + listOf(TimeMarkerFactory(n, dt, a)), cont
            )
        }

    fun setTangent(r: Rotation2d) =
        TrajectoryActionBuilder(this, tb.setTangent(r), n, lastPose, lastTangent, ms, cont)
    fun setTangent(r: Double) = setTangent(Rotation2d.exp(r))

    fun setReversed(reversed: Boolean) =
        TrajectoryActionBuilder(this, tb.setReversed(reversed), n, lastPose, lastTangent, ms, cont)

    @JvmOverloads
    fun turn(angle: Double, turnConstraintsOverride: TurnConstraints? = null): TrajectoryActionBuilder {
        val b = endTrajectory()
        val b2 = b.stopAndAdd(
            factory.fromTurn(
                TimeTurn(b.lastPose, angle, turnConstraintsOverride ?: baseTurnConstraints)
            )
        )
        val lastPose = Pose2d(b2.lastPose.trans, b2.lastPose.rot + angle)
        val lastTangent = b2.lastTangent + angle
        return TrajectoryActionBuilder(
            b2,
            TrajectoryBuilder(
                lastPose,
                eps,
                beginEndVel = 0.0,
                baseVelConstraint,
                baseAccelConstraint,
                resolution,
                poseMap
            ),
            b2.n, lastPose, lastTangent, b2.ms, b2.cont
        )
    }
    @JvmOverloads
    fun turnTo(heading: Rotation2d, turnConstraintsOverride: TurnConstraints? = null): TrajectoryActionBuilder {
        val b = endTrajectory()
        return b.turn(heading - b.lastPose.rot, turnConstraintsOverride)
    }
    @JvmOverloads
    fun turnTo(heading: Double, turnConstraintsOverride: TurnConstraints? = null) =
        turnTo(Rotation2d.exp(heading), turnConstraintsOverride)

    @JvmOverloads
    fun forward(
        dist: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.forward(dist, velConstraintOverride, accelConstraintOverride), n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun forwardConstantHeading(
        dist: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.forwardConstantHeading(dist, velConstraintOverride, accelConstraintOverride),
        n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun forwardLinearHeading(
        dist: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.forwardLinearHeading(
            dist, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun forwardLinearHeading(
        dist: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.forwardLinearHeading(
            dist, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun forwardSplineHeading(
        dist: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.forwardSplineHeading(
            dist, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun forwardSplineHeading(
        dist: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.forwardSplineHeading(
            dist, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToX(
            posX, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToXConstantHeading(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToXConstantHeading(
            posX, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToXLinearHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToXLinearHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToXSplineHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToXSplineHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToY(
            posY, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToYConstantHeading(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToYConstantHeading(
            posY, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToYLinearHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToYLinearHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToYSplineHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToYSplineHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineTo(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineTo(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineToConstantHeading(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineToConstantHeading(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineToLinearHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineToLinearHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineToSplineHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineToSplineHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPose, lastTangent, ms, cont
    )

    fun build() = endTrajectory().cont(SequentialAction())
}
