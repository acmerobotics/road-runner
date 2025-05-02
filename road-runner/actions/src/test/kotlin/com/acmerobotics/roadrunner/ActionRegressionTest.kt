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

            pw.println("negative time markers")
            printAction(
                base
                    .lineToX(20.0)
                    .afterTime(-0.5, LabelAction("A"))
                    .lineToXLinearHeading(30.0, Math.PI / 2)
                    .build()
            )
            printAction(
                base
                    .lineToX(10.0)
                    .lineToXLinearHeading(20.0, Math.PI / 2)
                    .afterTime(-0.5, LabelAction("A"))
                    .lineToX(30.0)
                    .build()
            )
            printAction(
                base
                    .lineToX(10.0)
                    .lineToXLinearHeading(20.0, Math.PI / 2)
                    .lineToX(30.0)
                    .afterTime(-0.5, LabelAction("A"))
                    .build()
            )
            printAction(
                base
                    .afterTime(-0.5, LabelAction("A"))
                    .lineToX(10.0)
                    .build()
            )
            printAction(
                base
                    .lineToX(10.0)
                    .lineToXLinearHeading(20.0, PI / 4)
                    .lineToX(30.0)
                    .afterTime(-100.0, LabelAction("A"))
                    .build()
            )
            printAction(
                base
                    .lineToX(10.0)
                    .lineToXLinearHeading(20.0, PI / 4)
                    .endTrajectory()
                    .lineToX(30.0)
                    .afterTime(-100.0, LabelAction("A"))
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
            pw.println()

            pw.println("negative disp markers")
            printAction(
                base
                    .lineToX(20.0)
                    .afterDisp(-2.5, LabelAction("A"))
                    .lineToXLinearHeading(30.0, Math.PI / 2)
                    .build()
            )
            printAction(
                base
                    .lineToX(10.0)
                    .lineToXLinearHeading(20.0, Math.PI / 2)
                    .afterDisp(-2.5, LabelAction("A"))
                    .lineToX(30.0)
                    .build()
            )
            printAction(
                base
                    .lineToX(10.0)
                    .lineToXLinearHeading(20.0, Math.PI / 2)
                    .lineToX(30.0)
                    .afterDisp(-2.5, LabelAction("A"))
                    .build()
            )
            printAction(
                base
                    .lineToX(10.0)
                    .lineToXLinearHeading(20.0, PI / 4)
                    .lineToX(30.0)
                    .afterDisp(-100.0, LabelAction("A"))
                    .build()
            )
            printAction(
                base
                    .lineToX(10.0)
                    .lineToXLinearHeading(20.0, PI / 4)
                    .endTrajectory()
                    .lineToX(30.0)
                    .afterDisp(-100.0, LabelAction("A"))
                    .build()
            )
            pw.println()
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
    @Strictfp
    fun testDispMarkerFailures() {
        assertFails {
            base
                .afterDisp(1.0, LabelAction("A"))
                .build()
        }

        assertFails {
            base
                .afterDisp(1.0, LabelAction("A"))
                .waitSeconds(10.0)
                .lineToX(10.0)
                .build()
        }

        assertFails {
            base
                .afterDisp(-2.5, LabelAction("A"))
                .lineToX(10.0)
                .build()
        }
    }
}
