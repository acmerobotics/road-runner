package com.acmerobotics.roadrunner.ftc;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class CancellationSamples {
    public static final class FollowTrajectoryAction implements Action {
        public FollowTrajectoryAction(TimeTrajectory t) {

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public static void setDrivePowers(PoseVelocity2d powers) {
    }

    // sample: actionsCancelAbruptly
    public class CancelableFollowTrajectoryAction implements Action {
        private final FollowTrajectoryAction action;
        private boolean cancelled = false;

        public CancelableFollowTrajectoryAction(TimeTrajectory t) {
            action = new FollowTrajectoryAction(t);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (cancelled) {
                setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                return false;
            }

            return action.run(telemetryPacket);
        }

        public void cancelAbruptly() {
            cancelled = true;
        }
    }
    // end sample

    static TimeTrajectory traj;

    // sample: cancelableActionOpMode
    public class CancelableActionOpMode extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            FtcDashboard dash = FtcDashboard.getInstance();

            CancelableFollowTrajectoryAction cancelableAction = new CancelableFollowTrajectoryAction(traj);
            while (opModeIsActive()) {
                TelemetryPacket packet = new TelemetryPacket();
                cancelableAction.preview(packet.fieldOverlay());
                if (!cancelableAction.run(packet)) {
                    break;
                }

                if (gamepad1.a) {
                    cancelableAction.cancelAbruptly();
                }

                dash.sendTelemetryPacket(packet);
            }
        }
    }
    // end sample

    // sample: cancelableActionOpMode2
    public class CancelableActionOpMode2 extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            FtcDashboard dash = FtcDashboard.getInstance();

            Servo servo = hardwareMap.servo.get("servo");

            CancelableFollowTrajectoryAction cancelableAction = new CancelableFollowTrajectoryAction(traj);
            Action sequentialAction = new SequentialAction(
                    cancelableAction,
                    new SleepAction(0.5),
                    new InstantAction(() -> servo.setPosition(0.5))
            );
            while (opModeIsActive()) {
                TelemetryPacket packet = new TelemetryPacket();
                sequentialAction.preview(packet.fieldOverlay());
                if (!sequentialAction.run(packet)) {
                    break;
                }

                if (gamepad1.a) {
                    cancelableAction.cancelAbruptly();
                }

                dash.sendTelemetryPacket(packet);
            }
        }
    }
    // end sample

    // sample: actionsFailover
    public class FailoverAction implements Action {
        private final Action mainAction;
        private final Action failoverAction;
        private boolean failedOver = false;

        public FailoverAction(Action mainAction, Action failoverAction) {
            this.mainAction = mainAction;
            this.failoverAction = failoverAction;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (failedOver) {
                return failoverAction.run(telemetryPacket);
            }

            return mainAction.run(telemetryPacket);
        }

        public void failover() {
            failedOver = true;
        }
    }
    // end sample

    CancellationSamples() {
        // sample: actionsFailoverExample
        Action a = new FailoverAction(
                new FollowTrajectoryAction(traj),
                new InstantAction(() -> setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)))
        );
        // end sample
    }
}
