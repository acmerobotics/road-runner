package com.acmerobotics.roadrunner.ftc;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import java.util.function.BooleanSupplier;

public class ActionCompositionSamples {
    // sample: actionsLambda
    public static Action waitUntil(BooleanSupplier ready) {
        return packet -> !ready.getAsBoolean();
    }
    // end sample

    // sample: actionsStartAndWait
    public static Action startAndWait(Runnable start, BooleanSupplier ready) {
        return new SequentialAction(
                new InstantAction(() -> start.run()),
                waitUntil(ready)
        );
    }
    // end sample

    // sample: actionsWrapper
    public static Action withTelemetry(String name, Action child) {
        return new Action() {
            @Override
            public boolean run(TelemetryPacket packet) {
                boolean running = child.run(packet);
                packet.put(name, running ? "running" : "finished");
                return running;
            }

            @Override
            public void preview(Canvas fieldOverlay) {
                child.preview(fieldOverlay);
            }
        };
    }
    // end sample

    // sample: actionsWaitForBoth
    public static Action waitForBoth(
            Action driveAction, Action mechanismAction, Action nextAction) {
        return new SequentialAction(
                new ParallelAction(driveAction, mechanismAction),
                nextAction
        );
    }
    // end sample
}
