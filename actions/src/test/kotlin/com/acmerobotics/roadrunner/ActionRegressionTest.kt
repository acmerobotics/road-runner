package com.acmerobotics.roadrunner

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import kotlin.math.PI
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFails

class TrajectoryAction(val t: TimeTrajectory) : Action {
    override fun run(p: TelemetryPacket): Action? {
        TODO("Not yet implemented")
    }

    override fun preview(c: Canvas) {
        TODO("Not yet implemented")
    }

    override fun toString() = "Trajectory"
}

class TurnAction(val t: TimeTurn) : Action {
    override fun run(p: TelemetryPacket): Action? {
        TODO("Not yet implemented")
    }

    override fun preview(c: Canvas) {
        TODO("Not yet implemented")
    }

    override fun toString() = "Turn"
}

class LabelAction(private val s: String) : Action {
    override fun run(p: TelemetryPacket): Action? {
        TODO("Not yet implemented")
    }

    override fun preview(c: Canvas) {
        TODO("Not yet implemented")
    }

    override fun toString() = s
}

class ActionRegressionTest {
    @Test
    fun testTrajectoryActionBuilder() {
        val base =
            TrajectoryActionBuilder(
                object : TrajectoryActionFactory {
                    override fun fromTrajectory(t: TimeTrajectory) = TrajectoryAction(t)
                    override fun fromTurn(t: TimeTurn) = TurnAction(t)
                },
                Pose2d(0.0, 0.0, PI / 2),
                1e-3,
                TurnConstraints(2.0, -1.0, 1.0),
                TranslationalVelConstraint(20.0),
                ProfileAccelConstraint(-10.0, 10.0),
                1e-2
            )

        assertEquals(
            "SequentialAction(actions=[TrajectoryAction])",
            base
                .forward(10.0)
                .build()
                .toString()
        )

        assertEquals(
            "SequentialAction(actions=[TrajectoryAction])",
            base
                .forward(10.0)
                .forward(10.0)
                .build()
                .toString(),
        )

        assertEquals(
            "SequentialAction(actions=[TrajectoryAction, SleepAction(dt=10.0), TrajectoryAction])",
            base
                .forward(10.0)
                .waitSeconds(10.0)
                .forward(10.0)
                .build()
                .toString()
        )

        assertEquals(
            "SequentialAction(actions=[TrajectoryAction, TrajectoryAction, TrajectoryAction])",
            base
                .forward(10.0)
                .forwardLinearHeading(10.0, Rotation2d.exp(1.57))
                .forward(10.0)
                .forward(10.0)
                .build()
                .toString()
        )

        assertEquals(
            "SequentialAction(actions=[TrajectoryAction, ParallelAction(actions=[SequentialAction(actions=" +
                "[TrajectoryAction, TrajectoryAction]), SequentialAction(actions=[SleepAction(dt=2.0), A])])])",
            base
                .forward(10.0)
                .forwardLinearHeading(10.0, Rotation2d.exp(1.57))
                .afterTime(2.0, LabelAction("A"))
                .forward(10.0)
                .forward(10.0)
                .build()
                .toString()
        )

        assertEquals(
            "SequentialAction(actions=[TrajectoryAction, ParallelAction(actions=[SequentialAction(actions=[" +
                "TrajectoryAction, TrajectoryAction]), SequentialAction(actions=[SleepAction(dt=2.0), A]), " +
                "SequentialAction(actions=[SleepAction(dt=3.499999999999996), B])])])",
            base
                .forward(10.0)
                .forwardLinearHeading(10.0, Rotation2d.exp(1.57))
                .afterTime(2.0, LabelAction("A"))
                .forward(5.0)
                .afterTime(1.5, LabelAction("B"))
                .forward(10.0)
                .build()
                .toString()
        )

        assertEquals(
            "ParallelAction(actions=[SequentialAction(actions=[]), SequentialAction(actions=[" +
                "SleepAction(dt=1.0), a])])",
            base
                .afterTime(1.0, LabelAction("a"))
                .build()
                .toString()
        )

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
                .forward(10.0)
                .build()
                .toString()
        }

        assertEquals(
            "ParallelAction(actions=[SequentialAction(actions=[TrajectoryAction]), SequentialAction(" +
                "actions=[SleepAction(dt=0.44721359549995765), A])])",
            base
                .afterDisp(1.0, LabelAction("A"))
                .forward(10.0)
                .build()
                .toString()
        )

        assertEquals(
            "ParallelAction(actions=[SequentialAction(actions=[TrajectoryAction]), SequentialAction(" +
                "actions=[SleepAction(dt=0.316227766016838), A])])",
            base
                .afterDisp(1.0, LabelAction("A"))
                .forward(0.25)
                .build()
                .toString()
        )

        assertEquals(
            "ParallelAction(actions=[SequentialAction(actions=[TrajectoryAction, TrajectoryAction]), " +
                "SequentialAction(actions=[SleepAction(dt=0.316227766016838), A])])",
            base
                .afterDisp(1.0, LabelAction("A"))
                .forward(0.25)
                .forwardLinearHeading(10.0, PI / 4)
                .build()
                .toString()
        )
    }
}
