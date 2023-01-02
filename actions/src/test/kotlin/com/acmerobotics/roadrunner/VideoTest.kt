package com.acmerobotics.roadrunner

import java.io.File
import kotlin.math.PI
import kotlin.test.assertFails

fun main() {
    writeVideo(
        File("videos/forward.mp4"),
        videoBuilder()
            .afterDisp(2.0, LabelAction("a"))
            .afterDisp(8.0, LabelAction("b"))
            .lineToX(12.0)
            .build()
    )

    assertFails {
        videoBuilder()
            .afterDisp(2.0, LabelAction("a"))
            .build()
    }

    writeVideo(
        File("videos/turn.mp4"),
        videoBuilder()
            .afterTime(0.25, LabelAction("a"))
            .turn(PI / 2)
            .build()
    )

    writeVideo(
        File("videos/forwardAutoRev.mp4"),
        videoBuilder()
            .lineToX(-12.0)
            .build()
    )

    writeVideo(
        File("videos/forwardAutoRevSpline.mp4"),
        videoBuilder()
            .lineToX(-12.0)
            .splineTo(Vector2d(-24.0, -12.0), -PI / 2)
            .build()
    )

    // TODO: setReversed() is broken....
    writeVideo(
        File("videos/forwardSplineRevSpline.mp4"),
        videoBuilder()
            .lineToX(-12.0)
            .splineTo(Vector2d(-24.0, -12.0), -PI / 2)
            .setReversed(false) // where did we reverse? *the forward call*
            .splineTo(Vector2d(-36.0, 0.0), PI)
            .build()
    )

    writeVideo(
        File("videos/forwardSplineTurnSpline.mp4"),
        videoBuilder()
            .lineToX(-12.0)
            .splineTo(Vector2d(-24.0, -12.0), -PI / 2)
            .turn(PI)
            .setReversed(true) // now we need to reverse after the turn
            .splineTo(Vector2d(-36.0, 0.0), PI)
            .build()
    )

    writeVideo(
        File("videos/forwardSplineTurn2Spline.mp4"),
        videoBuilder()
            .lineToX(-12.0)
            .afterDisp(5.0, LabelAction("first"))
            .splineTo(Vector2d(-24.0, -12.0), -PI / 2)
            .turn(PI / 2)
            .afterTime(0.5, LabelAction("second"))
            .turn(PI / 2)
            .setReversed(true) // now we need to reverse after the turn
            .splineTo(Vector2d(-36.0, 0.0), PI)
            .build()
    )

    writeVideo(
        File("videos/reverse.mp4"),
        videoBuilder()
            .lineToX(12.0)
            .setReversed(true)
            .splineTo(Vector2d(0.0, -12.0), -PI / 2)
            .build()
    )

    writeVideo(
        File("videos/changeTangent.mp4"),
        videoBuilder()
            .lineToX(12.0)
            .setTangent(-PI / 2)
            .splineTo(Vector2d(0.0, -12.0), PI)
            .build()
    )

    writeVideo(
        File("videos/changeTangent2.mp4"),
        videoBuilder()
            .lineToX(-12.0)
            .setTangent(-PI / 2)
            .splineTo(Vector2d(0.0, -12.0), 0.0)
            .build()
    )

    writeVideo(
        File("videos/originLoop.mp4"),
        videoBuilder()
            .lineToX(-12.0)
            .lineToX(0.0)
            .build()
    )
}
