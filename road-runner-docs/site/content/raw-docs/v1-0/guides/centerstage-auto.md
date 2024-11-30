---
title: Building a CENTERSTAGE Autonomous
---

# Building a CENTERSTAGE Autonomous

{{< hint info >}}
This is a community guide written by FTC Team 6051. Thanks for contributing to the docs!
{{< /hint >}}

After tuning, you will be ready to build your first auto routine with Roadrunner
1.0.X. Some parts of this process will feel familiar, but just like the tuning
guide, **read this page very carefully** to fully understand the logic behind
each step/declaration.

If you copy-and-paste the provided sample code, it will likely not work for your
robot, as the non-chassis actions described here are generic and will need to be
redefined for your specific mechanisms. The intent of this page is not to
provide you with runnable code out-of-the-box but instead break down the
process of writing an autonomous routine so that you may write your own. **It is
highly recommended that you code along with the creation process**.

## Step One: Imports

As with any autonomous, you will include a package statement and imports. For
this guide, we use the following imports and package:

```java
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
```

<!-- For teams not using encoders on
mechanisms, `DcMotor` should replace `DcMotorEx`.  -->

Of course, you may need additional imports depending on your robot hardware, or
you may not need all of the ones included here. Most notably, `import
com.acmerobotics.roadrunner.ParallelAction;` can be added to enable parallel
actions. Parallel actions are not used here, but are instantiated in essentially
the same way as sequential actions.

## Step Two: Define Auto Setup

As with all FTC autonomous modes, it is necessary to define the file as an
autonomous routine like so:
```java
@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {}
```
<!-- TODO: link to external docs -- maybe official FTC ones / book -->
If this step is confusing, we **strongly** recommend you read through the sample
Autonomous OpModes under the FTC Robot Controller, as this is not
Roadrunner-specific and will be essential to making any and all autos.

## Step Three: Instantiating Mechanism Classes
For each mechanism *not* including your drivetrain, create a new class defining
the hardware involved in the mechanism. This hardware will form the basis for
methods that return *actions*, which we will put together to make an autonomous
routine.

The following classes instantiate a `DcMotor`-driven, encoder-controlled, linear
lift system and a simple servo claw.
```java
// lift class
public class Lift {
    private DcMotorEx lift;

    public Lift(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}

// claw class
public class Claw {
    private Servo claw;

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
    }
}
```
This will ideally feel familiar, since non-RR autonomous routines also
pull from a hardware map in this way.

## Step Four: Adding Actions to Mechanisms

For each new mechanism class, we are going to add *actions*. This is where the
process becomes RR-specific, so if this is your first time designing a
1.0.X RR autonomous, **read carefully.**

Starting with the lift, we are going to create a `LiftUp` class that moves the
lift based on encoder position.

```java
public class LiftUp implements Action {
    // checks if the lift motor has been powered on
    private boolean initialized = false;

    // actions are formatted via telemetry packets as below
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        // powers on motor, if it is not on
        if (!initialized) {
            lift.setPower(0.8);
            initialized = true;
        }

        // checks lift's current position
        double pos = lift.getCurrentPosition();
        packet.put("liftPos", pos);
        if (pos < 3000.0) {
            // true causes the action to rerun
            return true;
        } else {
            // false stops action rerun
            lift.setPower(0);
            return false;
        }
        // overall, the action powers the lift until it surpasses
        // 3000 encoder ticks, then powers it off
    }
}
```

We can now create a method that instantiates a `LiftUp` action for convenience.
```java
public Action liftUp() {
    return new LiftUp();
}
```
Now, let's do the same for the `LiftDown`, `OpenClaw`, and `CloseClaw` actions.
```java
// within the Lift class
public class LiftDown implements Action {
    private boolean initialized = false;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            lift.setPower(-0.8);
            initialized = true;
        }

        double pos = lift.getCurrentPosition();
        packet.put("liftPos", pos);
        if (pos > 100.0) {
            return true;
        } else {
            lift.setPower(0);
            return false;
        }
    }
}

public Action liftDown() {
    return new LiftDown();
}

// within the Claw class
public class CloseClaw implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        claw.setPosition(0.55);
        return false;
    }
}
public Action closeClaw() {
    return new CloseClaw();
}

public class OpenClaw implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        claw.setPosition(1.0);
        return false;
    }
}
public Action openClaw() {
    return new OpenClaw();
}
```
Great! Our mechanisms are now ready to access from the `runOpMode()` method.

## Step Five: `runOpMode()` and Class Instances
After the mechanism classes, but still inside the `BlueSideTestAuto` class, we add
the following:
```java
@Override
public void runOpMode() {
    // instantiate your MecanumDrive at a particular pose.
    Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
    // make a Claw instance
    Claw claw = new Claw(hardwareMap);
    // make a Lift instance
    Lift lift = new Lift(hardwareMap);
}
```

{{< hint warning >}}
Make _sure_ your `MecanumDrive` is instantiated at the correct pose.
If you end up using `lineToX()`, `lineToY()`, `strafeTo()`, `splineTo()`, or any
of their variants in your code, if the initial pose is wrong, all future
movements will be thrown off.
{{< /hint >}}

## Step Six: Placehold for Vision
You will likely create your own vision pipeline to find the custom element your
team has created. Since vision is out of the scope of this tutorial, we are
going to set a vision output like so:
```java
// vision here that outputs position
int visionOutputPosition = 1;
```
Assuming that you have vision, you will want three trajectories for you to
choose from.

## Step Seven: Actually Building the Actions
These trajectories only cover the surface-level of what RR has to offer,
but they do offer valuable insight into trajectory-building structure. Note that
for `lineToX()` and `lineToY()` methods, since the current heading will be used to
construct the trajectory line, the heading normally cannot be orthogonal to the
line direction.

If the heading needs to remain orthogonal, you can use
`setTangent(Math.toRadians(*angle in degrees*))` to set a tangent line for the
robot to build a trajectory along.

Without further ado, we define a path for `trajectoryAction1`:
```java
// actionBuilder builds from the drive steps passed to it
TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
        .lineToYSplineHeading(33, Math.toRadians(0))
        .waitSeconds(2)
        .setTangent(Math.toRadians(90))
        .lineToY(48)
        .setTangent(Math.toRadians(0))
        .lineToX(32)
        .strafeTo(new Vector2d(44.5, 30))
        .turn(Math.toRadians(180))
        .lineToX(47.5)
        .waitSeconds(3);
```
Similarly, we can instantiate the other three drive actions:
```java
TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
        .lineToY(37)
        .setTangent(Math.toRadians(0))
        .lineToX(18)
        .waitSeconds(3)
        .setTangent(Math.toRadians(0))
        .lineToXSplineHeading(46, Math.toRadians(180))
        .waitSeconds(3)
        .build();
TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
        .lineToYSplineHeading(33, Math.toRadians(180))
        .waitSeconds(2)
        .strafeTo(new Vector2d(46, 30))
        .waitSeconds(3)
        .build();
Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
        .strafeTo(new Vector2d(48, 12))
        .build();
```
While the current set vision result means that trajectory actions 2 and 3 will
never be run, a dynamic vision result will allow them to be run.

## Step Eight: Other On-Init Actions

{{< hint warning >}}
If you implement these, your robot will need a "Moves on Initialization"
sticker.
{{< /hint >}}

All of the above work we did happens during the initialization of the robot. You
want all of your vision and path building to happen up here, because both of
those take a lot of time to initialize, and you don't want to lose auto runtime
to trajectory generation.

However, if you would like to add additional servo motions, you can do that by
running `Actions.runBlocking()` on the corresponding servo actions like so:
```java
// actions that need to happen on init; for instance, a claw tightening.
Actions.runBlocking(claw.closeClaw());
```

## Step Nine: The Initialization Limbo
Now, we enter the limbo between initialization completion and start. Many teams
will choose to continuously update vision during this time, and output telemetry
as shown below.
```java
while (!isStopRequested() && !opModeIsActive()) {
    int position = visionOutputPosition;
    telemetry.addData("Position during Init", position);
    telemetry.update();
}
int startPosition = visionOutputPosition;
telemetry.addData("Starting Position", startPosition);
telemetry.update();
waitForStart();
```

## Step Ten: Runtime!
We are now in the runtime! We always add the following to be able to stop the
robot if need be.
```java
if (isStopRequested()) return;
```
Now, we are going to do a simple vision-based trajectory selection as below:
```java
Action trajectoryActionChosen;
if (startPosition == 1) {
    trajectoryActionChosen = tab1.build();
} else if (startPosition == 2) {
    trajectoryActionChosen = tab2.build();
} else {
    trajectoryActionChosen = tab3.build();
}
```
Once that's handled, we are all ready to run our action sequence!
```java
Actions.runBlocking(
        new SequentialAction(
                trajectoryActionChosen,
                lift.liftUp(),
                claw.openClaw(),
                lift.liftDown(),
                trajectoryActionCloseOut
        )
);
```
Congratulations! Provided everything is configured correctly, you've just
written your first autonomous in Roadrunner 1.0.X! From here, it's all
customizing to your specific use case. With <3, Anya Levin (Team #6051, Quantum
Mechanics)

## Final Code
For anyone interested, here is the sample autonomous all put together!

```java
package org.firstinspires.ftc.teamcode.teleops;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {
    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.55);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        lift.liftUp(),
                        claw.openClaw(),
                        lift.liftDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}
```