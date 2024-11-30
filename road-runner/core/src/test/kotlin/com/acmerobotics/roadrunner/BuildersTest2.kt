package com.acmerobotics.roadrunner

import kotlin.math.PI
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFails

class BuildersTest2 {
    @Test
    fun testPosPathSeqBuilder() {
        val posBase = PositionPathSeqBuilder(
            Vector2d(0.0, 0.0),
            Rotation2d.exp(0.0),
            1e-6,
        )

        assertFails {
            posBase.build()
        }

        assertEquals(
            1,
            posBase
                .lineToX(12.0)
                .lineToX(24.0)
                .build()
                .size
        )

        assertEquals(
            2,
            posBase
                .lineToX(12.0)
                // automatic tangent reversal occurs here
                .lineToX(0.0)
                .build()
                .size
        )

        assertFails {
            posBase
                .lineToY(10.0)
                .build()
        }

        assertEquals(
            1,
            posBase
                .lineToX(-12.0)
                .build()
                .size
        )

        assertEquals(
            1,
            posBase
                .splineTo(Vector2d(24.0, -24.0), PI / 2)
                .splineTo(Vector2d(-48.0, 0.0), PI)
                .build()
                .size
        )

        // it might seem like this spline _should_ reverse, but there is no real tangent violation
        // splines demand manual handling of tangents
        assertEquals(
            1,
            posBase
                .splineTo(Vector2d(24.0, -24.0), PI / 2)
                .splineTo(Vector2d(0.0, 0.0), 0.0)
                .build()
                .size
        )

        // assertFails {
        //     posBase
        //         .splineTo(Vector2d(0.0, 0.0), 0.0)
        //         .build()
        // }

        assertEquals(
            1,
            posBase
                .lineToX(10.0)
                .splineTo(Vector2d(10.0, 0.0), 0.0)
                .build()
                .size
        )
    }

    @Test
    fun testPosePathSeqBuilder() {
        val path = PositionPathSeqBuilder(
            Vector2d(0.0, 0.0),
            Rotation2d.exp(0.0),
            1e-6,
        )
            .lineToX(10.0)
            .build()
            .first()

        val poseBase = PosePathSeqBuilder(path, PI)

        assertEquals(
            1,
            poseBase
                .tangentUntil(5.0)
                .tangentUntilEnd()
                .size
        )

        // this happens to work numerically but doesn't hold for every path
        assertEquals(
            1,
            poseBase
                .constantUntil(5.0)
                .tangentUntilEnd()
                .size
        )

        assertFails {
            poseBase
                .linearUntil(15.0, PI / 8)
        }

        assertFails {
            poseBase
                .linearUntil(15.0, PI / 8)
                .tangentUntilEnd()
        }

        // this worked before...
        assertEquals(
            1,
            poseBase
                .linearUntil(5.0, PI / 8)
                .splineUntilEnd(0.0)
                .size
        )

        // ...but now this also works
        assertEquals(
            1,
            poseBase
                .splineUntil(5.0, PI / 8)
                .linearUntilEnd(0.0)
                .size
        )

        // and this still works
        assertEquals(
            1,
            poseBase
                .splineUntil(5.0, PI / 8)
                .splineUntilEnd(0.0)
                .size
        )

        assertEquals(
            1,
            poseBase
                .linearUntil(2.0, PI / 4)
                .splineUntil(5.0, PI / 8)
                .linearUntilEnd(0.0)
                .size
        )

        assertEquals(
            2,
            poseBase
                .linearUntil(2.0, PI / 4)
                .linearUntilEnd(0.0)
                .size
        )

        assertEquals(
            2,
            poseBase
                .constantUntil(5.0)
                .linearUntilEnd(PI / 2)
                .size
        )
    }
}
