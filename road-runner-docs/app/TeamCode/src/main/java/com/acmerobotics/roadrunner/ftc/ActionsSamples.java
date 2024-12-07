package com.acmerobotics.roadrunner.ftc;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;

public class ActionsSamples {
    public static class TodoAction implements Action {
        @Override
        public boolean run(@NotNull TelemetryPacket p) {
            return false;
        }
    }

    // sample: actionsDecl
    public class Drive {
        public Action followTrajectory(Trajectory t) {
            return new TodoAction();
        }

        public Action turn(double angle) {
            return new TodoAction();
        }

        public Action moveToPoint(double x, double y) {
            return new TodoAction();
        }
    }

    public class Shooter {
        public Action spinUp() {
            return new TodoAction();
        }

        public Action fireBall() {
            return new TodoAction();
        }

        public Action loadBall() {
            return new TodoAction();
        }
    }
    // end sample

    ActionsSamples() {
        // sample: actionsMoveToPoint
        Drive drive = new Drive();
        Actions.runBlocking(drive.moveToPoint(10, 20));
        // end sample

        final Trajectory shootingTraj = null;
        final Shooter shooter = new Shooter();
        // sample: actionsCompositeAction
        Actions.runBlocking(new SequentialAction(
                drive.turn(Math.PI / 2),
                new ParallelAction(
                        drive.followTrajectory(shootingTraj),
                        new SequentialAction(
                                shooter.spinUp(),
                                shooter.fireBall()
                        )
                )
        ));
        // end sample
    }

    class ShooterComplete {
        // sample: actionsShooterComplete
        public class Shooter {
            private DcMotorEx motor;

            public Shooter(HardwareMap hardwareMap) {
                motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
            }

            public class SpinUp implements Action {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        motor.setPower(0.8);
                        initialized = true;
                    }

                    double vel = motor.getVelocity();
                    packet.put("shooterVelocity", vel);
                    return vel < 10_000.0;
                }
            }

            public Action spinUp() {
                return new SpinUp();
            }
        }

        public class ShooterOpMode extends LinearOpMode {
            @Override
            public void runOpMode() throws InterruptedException {
                Shooter shooter = new Shooter(hardwareMap);

                waitForStart();

                Actions.runBlocking(shooter.spinUp());
            }
        }
        // end sample
    }

    static class ShooterRefactor {
        // sample: actionsShooterRefactor
        class Shooter {
            private DcMotorEx motor;

            public Shooter(HardwareMap hardwareMap) {
                motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
            }

            public Action spinUp() {
                return new Action() {
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (!initialized) {
                            motor.setPower(0.8);
                            initialized = true;
                        }

                        double vel = motor.getVelocity();
                        packet.put("shooterVelocity", vel);
                        return vel < 10_000.0;
                    }
                };
            }
        }
        // end sample
    }
}
