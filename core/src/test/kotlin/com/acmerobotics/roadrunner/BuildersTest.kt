package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.sqrt
import kotlin.random.Random
import kotlin.test.assertEquals

class BuildersTest {
    @Test
    fun test() {
        val posPath = PositionPathBuilder(
            Position2(0.0, 0.0),
            Rotation2.exp(0.0)
        )
            .splineTo(
                Position2(15.0, 15.0),
                Rotation2.exp(PI),
            )
            .splineTo(
                Position2(5.0, 35.0),
                Rotation2.exp(PI / 3),
            )
            .build()

        val posePath = PosePathBuilder(posPath, Rotation2.exp(0.0))
            .tangentTo((posPath.offsets[0] + posPath.offsets[1]) / 2)
            .splineTo(posPath.length, Rotation2.exp(-PI / 3))
            .build()

        println(posPath.offsets)
        println(posePath.offsets)
    }
}
