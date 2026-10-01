package com.acmerobotics.roadrunner

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import java.io.File
import java.io.PrintWriter
import java.io.StringWriter
import kotlin.math.PI
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFails
import kotlin.test.assertFalse
import kotlin.test.assertIs

class TrajectoryAction(val t: TimeTrajectory) : Action {
    override fun run(p: TelemetryPacket): Boolean {
        TODO("Not yet implemented")
    }

    override fun preview(fieldOverlay: Canvas) {
        TODO("Not yet implemented")
    }

    override fun toString() = "Trajectory"
}

class TurnAction(val t: TimeTurn) : Action {
    override fun run(p: TelemetryPacket): Boolean {
        TODO("Not yet implemented")
    }

    override fun preview(fieldOverlay: Canvas) {
        TODO("Not yet implemented")
    }

    override fun toString() = "Turn"
}

class LabelAction(val s: String) : Action {
    override fun run(p: TelemetryPacket): Boolean {
        TODO("Not yet implemented")
    }

    override fun preview(fieldOverlay: Canvas) {
        TODO("Not yet implemented")
    }

    override fun toString() = s
}

fun sexpFromAction(a: Action): Sexp =
    when (a) {
        is TrajectoryAction -> Sexp.Atom("traj")
        is TurnAction -> Sexp.Atom("turn")
        is LabelAction -> Sexp.Atom(a.s)
        is SleepAction -> Sexp.list(Sexp.Atom("sleep"), Sexp.Atom(String.format("%.10f", a.dt)))
        is SequentialAction -> Sexp.list(listOf(Sexp.Atom("seq")) + a.initialActions.map(::sexpFromAction))
        is ParallelAction -> Sexp.list(listOf(Sexp.Atom("par")) + a.initialActions.map(::sexpFromAction))
        is NullAction -> Sexp.Atom("null")
        else -> Sexp.Atom("unk")
    }

class ActionRegressionTest {
    companion object {
        @JvmField
        val base =
            TrajectoryActionBuilder(
                { TurnAction(it) },
                { TrajectoryAction(it) },
                TEST_TRAJECTORY_BUILDER_PARAMS,
                Pose2d(0.0, 0.0, 0.0),
                0.0,
                TurnConstraints(2.0, -1.0, 1.0),
                TranslationalVelConstraint(20.0),
                ProfileAccelConstraint(-10.0, 10.0),
            )

        const val UPDATE = false
    }

    @Test
    @Strictfp
    fun test() {
        val sw = StringWriter()
        PrintWriter(sw).use { pw ->
            val config = Config.create(color = false)
            fun printAction(a: Action) {
                pw.println(prettyPrint(sexpFromAction(a), config))
            }

            pw.println("basic")
            printAction(
                base
                    .lineToX(10.0)
                    .build()
            )
            printAction(
                base
                    .lineToX(10.0)
                    .lineToX(20.0)
                    .build()
            )
            pw.println()

            pw.println("continuity")
            printAction(
                base
                    .lineToX(10.0)
                    .waitSeconds(10.0)
                    .lineToX(20.0)
                    .build()
            )
            printAction(
                base
                    .lineToX(10.0)
                    .lineToXLinearHeading(20.0, Rotation2d.exp(1.57))
                    .lineToX(30.0)
                    .lineToX(40.0)
                    .build()
            )
            pw.println()

            pw.println("time markers")
            printAction(
                base
                    .lineToX(10.0)
                    .lineToXLinearHeading(20.0, Rotation2d.exp(1.57))
                    .afterTime(2.0, LabelAction("A"))
                    .lineToX(25.0)
                    .lineToX(30.0)
                    .build()
            )
            printAction(
                base
                    .lineToX(10.0)
                    .lineToXLinearHeading(20.0, Rotation2d.exp(1.57))
                    .afterTime(2.0, LabelAction("A"))
                    .lineToX(25.0)
                    .afterTime(1.5, LabelAction("B"))
                    .lineToX(35.0)
                    .build()
            )
            printAction(
                base
                    .afterTime(1.0, LabelAction("a"))
                    .build()
            )
            pw.println()

            pw.println("disp markers")
            printAction(
                base
                    .afterDisp(1.0, LabelAction("A"))
                    .lineToX(10.0)
                    .build()
            )
            printAction(
                base
                    .afterDisp(1.0, LabelAction("A"))
                    .lineToX(0.25)
                    .build()
            )
            printAction(
                base
                    .afterDisp(1.0, LabelAction("A"))
                    .lineToX(0.25)
                    .lineToXLinearHeading(10.25, PI / 4)
                    .build()
            )
        }

        val f = File("values.txt")
        if (UPDATE) {
            f.writeText(sw.toString())
        } else {
            val expected = f.readText()
            assertEquals(expected, sw.toString())
        }

        assertFalse(UPDATE)
    }

    @Test
    fun testTimeMarkerBeforeAndAfterMovement() {
        val marker = LabelAction("marker")
        val before = assertIs<ParallelAction>(
            base.afterTime(0.0, marker).lineToX(10.0).build()
        )
        assertIs<TrajectoryAction>(before.initialActions[0])
        val beforeMarker = assertIs<SequentialAction>(before.initialActions[1])
        assertEquals(listOf(SleepAction(0.0), marker), beforeMarker.initialActions)

        val after = assertIs<ParallelAction>(
            base.lineToX(10.0).afterTime(0.0, marker).build()
        )
        val trajectory = assertIs<TrajectoryAction>(after.initialActions[0])
        val afterMarker = assertIs<SequentialAction>(after.initialActions[1])
        assertEquals(trajectory.t.duration, assertIs<SleepAction>(afterMarker.initialActions[0]).dt, 1e-6)
        assertEquals(marker, afterMarker.initialActions[1])
    }

    @Test
    fun testTimeMarkerAfterTurn() {
        val marker = LabelAction("marker")
        val action = assertIs<SequentialAction>(
            base.turn(PI / 2).afterTime(0.5, marker).turn(PI / 2).build()
        )
        assertEquals(2, action.initialActions.size)
        assertIs<TurnAction>(action.initialActions[0])
        val parallel = assertIs<ParallelAction>(action.initialActions[1])
        assertIs<TurnAction>(parallel.initialActions[0])
        val markerSequence = assertIs<SequentialAction>(parallel.initialActions[1])
        assertEquals(listOf(SleepAction(0.5), marker), markerSequence.initialActions)
    }

    @Test
    fun testFractionalWaitsWithTimeMarker() {
        val lift = LabelAction("lift")
        val outtake = LabelAction("outtake")
        val reset = LabelAction("reset")
        val action = assertIs<ParallelAction>(
            base
                .afterTime(0.0, lift)
                .strafeTo(Vector2d(40.0, 32.0))
                .waitSeconds(0.9)
                .stopAndAdd(outtake)
                .waitSeconds(0.9)
                .stopAndAdd(reset)
                .waitSeconds(4.0)
                .build()
        )
        val sequence = assertIs<SequentialAction>(action.initialActions[0])
        assertIs<TrajectoryAction>(sequence.initialActions[0])
        assertEquals(
            listOf(SleepAction(0.9), outtake, SleepAction(0.9), reset, SleepAction(4.0)),
            sequence.initialActions.drop(1)
        )
        val markerSequence = assertIs<SequentialAction>(action.initialActions[1])
        assertEquals(listOf(SleepAction(0.0), lift), markerSequence.initialActions)
    }

    @Test
    @Strictfp
    fun testDispMarkFailures() {
        assertFails {
            base
                .afterDisp(1.0, LabelAction("A"))
                .build()
                .toString()
        }

        assertFails {
            base
                .afterDisp(1.0, LabelAction("A"))
                .waitSeconds(10.0)
                .lineToX(10.0)
                .build()
                .toString()
        }
    }
}
