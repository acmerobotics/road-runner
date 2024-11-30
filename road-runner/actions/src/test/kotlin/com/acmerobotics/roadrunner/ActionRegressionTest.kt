package com.acmerobotics.roadrunner

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import kotlin.math.PI
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFails

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

class LabelAction(private val s: String) : Action {
    override fun run(p: TelemetryPacket): Boolean {
        TODO("Not yet implemented")
    }

    override fun preview(fieldOverlay: Canvas) {
        TODO("Not yet implemented")
    }

    override fun toString() = s
}

class ActionRegressionTest {
    @Test
    @Strictfp
    fun testTrajectoryActionBuilder() {
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

        assertEquals(
            "SequentialAction(initialActions=[Trajectory])",
            base
                .lineToX(10.0)
                .build()
                .toString()
        )

        assertEquals(
            "SequentialAction(initialActions=[Trajectory])",
            base
                .lineToX(10.0)
                .lineToX(20.0)
                .build()
                .toString(),
        )

        assertEquals(
            "SequentialAction(initialActions=[Trajectory, SleepAction(dt=10.0), Trajectory])",
            base
                .lineToX(10.0)
                .waitSeconds(10.0)
                .lineToX(20.0)
                .build()
                .toString()
        )

        assertEquals(
            "SequentialAction(initialActions=[Trajectory, Trajectory, Trajectory])",
            base
                .lineToX(10.0)
                .lineToXLinearHeading(20.0, Rotation2d.exp(1.57))
                .lineToX(30.0)
                .lineToX(40.0)
                .build()
                .toString()
        )

        assertEquals(
            "SequentialAction(initialActions=[Trajectory, ParallelAction(initialActions=[" +
                "SequentialAction(initialActions=" +
                "[Trajectory, Trajectory]), SequentialAction(initialActions=[SleepAction(dt=2.0), A])])])",
            base
                .lineToX(10.0)
                .lineToXLinearHeading(20.0, Rotation2d.exp(1.57))
                .afterTime(2.0, LabelAction("A"))
                .lineToX(25.0)
                .lineToX(30.0)
                .build()
                .toString()
        )

        assertEquals(
            "SequentialAction(initialActions=[Trajectory, ParallelAction(initialActions=[" +
                "SequentialAction(initialActions=[" +
                "Trajectory, Trajectory]), SequentialAction(initialActions=[SleepAction(dt=2.0), A]), " +
                "SequentialAction(initialActions=[SleepAction(dt=3.499999999999999), B])])])",
            base
                .lineToX(10.0)
                .lineToXLinearHeading(20.0, Rotation2d.exp(1.57))
                .afterTime(2.0, LabelAction("A"))
                .lineToX(25.0)
                .afterTime(1.5, LabelAction("B"))
                .lineToX(35.0)
                .build()
                .toString()
        )

        assertEquals(
            "ParallelAction(initialActions=[SequentialAction(initialActions=[]), " +
                "SequentialAction(initialActions=[" +
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
                .lineToX(10.0)
                .build()
                .toString()
        }

        assertEquals(
            "ParallelAction(initialActions=[SequentialAction(initialActions=[Trajectory]), " +
                "SequentialAction(" +
                "initialActions=[SleepAction(dt=0.44721359549995804), A])])",
            base
                .afterDisp(1.0, LabelAction("A"))
                .lineToX(10.0)
                .build()
                .toString()
        )

        assertEquals(
            "ParallelAction(initialActions=[SequentialAction(initialActions=[Trajectory]), " +
                "SequentialAction(" +
                "initialActions=[SleepAction(dt=0.4472135954999579), A])])",
            base
                .afterDisp(1.0, LabelAction("A"))
                .lineToX(0.25)
                .build()
                .toString()
        )

        assertEquals(
            "ParallelAction(initialActions=[SequentialAction(initialActions=[Trajectory, " +
                "Trajectory]), " +
                "SequentialAction(initialActions=[SleepAction(dt=0.4472135954999579), A])])",
            base
                .afterDisp(1.0, LabelAction("A"))
                .lineToX(0.25)
                .lineToXLinearHeading(10.25, PI / 4)
                .build()
                .toString()
        )
    }
}
