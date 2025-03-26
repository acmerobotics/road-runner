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
    }

    @Test
    @Strictfp
    fun testTrajectoryActionBuilder() {
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
    }

    @Test
    @Strictfp
    fun testTrajectoryContinuity() {
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
    }

    @Test
    @Strictfp
    fun testTrajectoryTimeMarkers() {
        assertEquals(
            "SequentialAction(initialActions=[Trajectory, Trajectory, ParallelAction(initialActions=[" +
                "SequentialAction(initialActions=" +
                "[Trajectory]), SequentialAction(initialActions=[SleepAction(dt=2.0), A])])])",
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
            "SequentialAction(initialActions=[Trajectory, Trajectory, ParallelAction(initialActions=[" +
                "SequentialAction(initialActions=[" +
                "Trajectory]), SequentialAction(initialActions=[SleepAction(dt=2.0), A]), " +
                "SequentialAction(initialActions=[SleepAction(dt=2.5000000000000004), B])])])",
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
                "SleepAction(dt=1.0), A])])",
            base
                .afterTime(1.0, LabelAction("A"))
                .build()
                .toString()
        )
    }

    @Test
    @Strictfp
    fun testTrajectoryTimeNegativeMarkers() {
        assertEquals(
            "ParallelAction(initialActions=[" +
                    "SequentialAction(initialActions=[Trajectory, Trajectory]), " +
                    "SequentialAction(initialActions=[SleepAction(dt=2.32842712474619), A])" +
                    "])",
            base
                .beforeEndTime(0.5, LabelAction("A"))
                .lineToX(20.0)
                .lineToXLinearHeading(30.0, Math.PI / 2)
                .build()
                .toString()
        )

        assertEquals(
            "SequentialAction(initialActions=[Trajectory, ParallelAction(initialActions=[" +
                    "SequentialAction(initialActions=[Trajectory, Trajectory]), " +
                    "SequentialAction(initialActions=[SleepAction(dt=1.4999999999999993), A])" +
                    "])])",
            base
                .lineToX(10.0)
                .beforeEndTime(0.5, LabelAction("A"))
                .lineToXLinearHeading(20.0, Math.PI / 2)
                .lineToX(30.0)
                .build()
                .toString(),
        )

        assertEquals(
            "SequentialAction(initialActions=[Trajectory, Trajectory, ParallelAction(initialActions=[" +
                    "SequentialAction(initialActions=[Trajectory]), " +
                    "SequentialAction(initialActions=[SleepAction(dt=1.5000000000000009), A])" +
                    "])])",
            base
                .lineToX(10.0)
                .lineToXLinearHeading(20.0, Math.PI / 2)
                .beforeEndTime(0.5, LabelAction("A"))
                .lineToX(30.0)
                .build()
                .toString()
        )
    }

    @Test
    @Strictfp
    fun testTrajectoryDispMarkers() {
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
                "initialActions=[SleepAction(dt=0.316227766016838), A])])",
            base
                .afterDisp(1.0, LabelAction("A"))
                .lineToX(0.25)
                .build()
                .toString()
        )

        assertEquals(
            "ParallelAction(initialActions=[SequentialAction(initialActions=[Trajectory, " +
                "Trajectory]), " +
                "SequentialAction(initialActions=[SleepAction(dt=0.316227766016838), A])])",
            base
                .afterDisp(1.0, LabelAction("A"))
                .lineToX(0.25)
                .lineToXLinearHeading(10.25, PI / 4)
                .build()
                .toString()
        )
    }

    @Test
    @Strictfp
    fun testTrajectoryDispNegativeMarkers() {
        assertEquals(
            "ParallelAction(initialActions=[" +
                    "SequentialAction(initialActions=[Trajectory, Trajectory]), " +
                    "SequentialAction(initialActions=[SleepAction(dt=2.121320343559643), A])" +
                    "])",
            base
                .beforeEndDisp(2.5, LabelAction("A"))
                .lineToX(20.0)
                .lineToXLinearHeading(30.0, Math.PI / 2)
                .build()
                .toString()
        )

        assertEquals(
            "SequentialAction(initialActions=[Trajectory, ParallelAction(initialActions=[" +
                    "SequentialAction(initialActions=[Trajectory, Trajectory]), " +
                    "SequentialAction(initialActions=[SleepAction(dt=1.2928932188134519), A])" +
                    "])])",
            base
                .lineToX(10.0)
                .beforeEndDisp(2.5, LabelAction("A"))
                .lineToXLinearHeading(20.0, Math.PI / 2)
                .lineToX(30.0)
                .build()
                .toString(),
        )

        assertEquals(
            "SequentialAction(initialActions=[Trajectory, Trajectory, ParallelAction(initialActions=[" +
                    "SequentialAction(initialActions=[Trajectory]), " +
                    "SequentialAction(initialActions=[SleepAction(dt=1.2928932188134532), A])" +
                    "])])",
            base
                .lineToX(10.0)
                .lineToXLinearHeading(20.0, Math.PI / 2)
                .beforeEndDisp(2.5, LabelAction("A"))
                .lineToX(30.0)
                .build()
                .toString()
        )
    }
}
