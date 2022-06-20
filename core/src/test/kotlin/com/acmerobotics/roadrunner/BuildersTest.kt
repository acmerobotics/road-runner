package com.acmerobotics.roadrunner

import org.knowm.xchart.QuickChart
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.theme.MatlabTheme
import kotlin.math.PI
import kotlin.random.Random
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFails

fun chartSpline(q: QuinticSpline1): XYChart {
    val ts = range(0.0, 1.0, 1000)
    val xs = ts.map { q[it, 3] }

    return QuickChart.getChart(
        "Spline", "t", "",
        arrayOf("x", "dx", "d2x", "dx num", "d2x num"),
        ts.toDoubleArray(),
        arrayOf(
            xs.map { it[0] }.toDoubleArray(),
            xs.map { it[1] }.toDoubleArray(),
            xs.map { it[2] }.toDoubleArray(),
            numericalDerivative(xs.map { it[0] }, 1.0 / 999).toDoubleArray(),
                numericalDerivative(xs.map { it[1] }, 1.0 / 999).toDoubleArray(),
                )
            ).also {
                it.styler.theme = MatlabTheme()
            }
        }

        fun chartSplineExpLog(q: QuinticSpline1): XYChart {
            val ts = range(0.0, 1.0, 1000)
            val xs = ts.map { (Rotation2Dual.exp(q[it, 3]) + 0.3).log() }

            return QuickChart.getChart(
                "Spline", "t", "",
                arrayOf("x", "dx", "d2x"),
                ts.toDoubleArray(),
                arrayOf(
                    xs.map { it[0] }.toDoubleArray(),
                    xs.map { it[1] }.toDoubleArray(),
                    xs.map { it[2] }.toDoubleArray(),
                )
            ).also {
                it.styler.theme = MatlabTheme()
            }
        }

        fun <Param> chartPosPath(posPath: PositionPath<Param>): XYChart {
            val params = range(-1.0, posPath.length + 1.0, 1000)
            val positions = params.map { posPath[it, 4] }

            return QuickChart.getChart(
                "Path", "param", "",
                arrayOf(
//                "x", "y",
                    "x'", "y'", "x''", "y''",
                    "x'''", "y'''",
                ),
                params.toDoubleArray(),
                arrayOf(
//                positions.map { it.x[0] }.toDoubleArray(),
//                positions.map { it.y[0] }.toDoubleArray(),
                    positions.map { it.x[1] }.toDoubleArray(),
                    positions.map { it.y[1] }.toDoubleArray(),
                    positions.map { it.x[2] }.toDoubleArray(),
                    positions.map { it.y[2] }.toDoubleArray(),
                    positions.map { it.x[3] }.toDoubleArray(),
                    positions.map { it.y[3] }.toDoubleArray(),
                )
            )
        }

        fun chartPosePath(posePath: PosePath): XYChart {
            val params = range(-1.0, posePath.length + 1.0, 1000)
            val poses = params.map { posePath[it, 3] }

            return QuickChart.getChart(
                "Path", "param", "",
                arrayOf(
//                "x", "y", "theta",
                    "x'", "y'", "theta'",
                    "x''", "y''", "theta''",
                ),
                params.toDoubleArray(),
                arrayOf(
//            poses.map { it.translation.x[0] }.toDoubleArray(),
//            poses.map { it.translation.y[0] }.toDoubleArray(),
//            poses.map { it.rotation.log()[0] }.toDoubleArray(),
                    poses.map { it.translation.x[1] }.toDoubleArray(),
                    poses.map { it.translation.y[1] }.toDoubleArray(),
                    poses.map { it.rotation.velocity()[0] }.toDoubleArray(),
                    poses.map { it.translation.x[2] }.toDoubleArray(),
                    poses.map { it.translation.y[2] }.toDoubleArray(),
                    poses.map { it.rotation.velocity()[1] }.toDoubleArray(),
                )
            )
        }

        fun chartPosePathHeading(posePath: PosePath): XYChart {
            val params = range(-1.0, posePath.length + 1.0, 1000)
            val poses = params.map { posePath[it, 3] }

            return QuickChart.getChart(
                "Path", "param", "",
                arrayOf(
//                "x", "y",
                    "theta",
                    "theta'",
                    "theta''",
                ),
                params.toDoubleArray(),
                arrayOf(
//            poses.map { it.translation.x[0] }.toDoubleArray(),
//            poses.map { it.translation.y[0] }.toDoubleArray(),
                    poses.map { it.rotation.log()[0] }.toDoubleArray(),
                    poses.map { it.rotation.velocity()[0] }.toDoubleArray(),
                    poses.map { it.rotation.velocity()[1] }.toDoubleArray(),
                )
            )
        }

        class BuildersTest {
            @Test
            fun testForward() {
                val r = Random.Default
                repeat(100) {
                    val beginPos = Position2(r.nextDouble(), r.nextDouble())
                    val beginTangent = Rotation2.exp(r.nextDouble())

                    val mag = r.nextDouble()
                    val endPos = beginPos + beginTangent.vec() * mag

                    val posPath = PositionPathBuilder(beginPos, beginTangent)
                        .forward(mag)
                        .build()

                    assertEquals(beginPos.x, posPath.begin(1).value().x, 1e-6)
                    assertEquals(beginPos.y, posPath.begin(1).value().y, 1e-6)
                    assertEquals(0.0, beginTangent - posPath.begin(2).tangent().value(), 1e-6)

                    assertEquals(endPos.x, posPath.end(1).value().x, 1e-6)
                    assertEquals(endPos.y, posPath.end(1).value().y, 1e-6)
                    assertEquals(0.0, beginTangent - posPath.end(2).tangent().value(), 1e-6)
                }
            }

            @Test
            fun testSplineTo() {
                val r = Random.Default
                repeat(100) {
                    val beginPos = Position2(r.nextDouble(), r.nextDouble())
                    val beginTangent = Rotation2.exp(r.nextDouble())
                    val endPos = Position2(r.nextDouble(), r.nextDouble())
                    val endTangent = Rotation2.exp(r.nextDouble())

                    val posPath = PositionPathBuilder(beginPos, beginTangent)
                        .splineTo(endPos, endTangent)
                        .build()

                    assertEquals(beginPos.x, posPath.begin(1).value().x, 1e-6)
                    assertEquals(beginPos.y, posPath.begin(1).value().y, 1e-6)
                    assertEquals(0.0, beginTangent - posPath.begin(2).tangent().value(), 1e-6)

                    assertEquals(endPos.x, posPath.end(1).value().x, 1e-6)
                    assertEquals(endPos.y, posPath.end(1).value().y, 1e-6)
                    assertEquals(0.0, endTangent - posPath.end(2).tangent().value(), 1e-6)
                }
            }

            @Test
            fun testTangentHeading() {
                val posPath = PositionPathBuilder(
                    Position2(0.0, 0.0),
                    Rotation2.exp(0.0)
                )
                    .splineTo(
                        Position2(15.0, 15.0),
                        Rotation2.exp(PI),
                    )
                    .build()

                val posePath = PosePathBuilder(posPath, Rotation2.exp(PI))
                    .tangentUntilEnd()

                saveChart("poseBuilder/tangent", chartPosePathHeading(posePath))
            }

            @Test
            fun testSplineHeading() {
                val posPathPre = PositionPathBuilder(
                    Position2(0.0, 0.0),
                    Rotation2.exp(0.0)
                )
                    .splineTo(
                        Position2(15.0, 15.0),
                        Rotation2.exp(PI),
                    )
//            .lineTo(
//                Position2(100.0, 0.0)
//            )
                    .build()

                val posPath = PositionPathBuilder(
                    Position2(0.0, 0.0),
                    Rotation2.exp(0.0)
                )
                    .splineTo(
                        (Vector2(15.0, 15.0) / posPathPre.length).bind(),
                        Rotation2.exp(PI),
                    )
                    .build()

                val posePath = PosePathBuilder(posPath, Rotation2.exp(PI))
                    .splineUntilEnd(Rotation2.exp(-PI / 3))

                saveChart("poseBuilder/spline", chartPosePathHeading(posePath))
                saveChart(
                    "poseBuilder/spline2",
                    chartSpline(
                        (
                            (
                                (posePath as CompositePosePath).paths[0]
                                    as HeadingPosePath
                                ).headingPath
                                as SplineHeadingPath
                            ).spline
                    )
                )
                saveChart(
                    "poseBuilder/spline3",
                    chartSplineExpLog(
                        (
                            (
                                posePath.paths[0]
                                    as HeadingPosePath
                                ).headingPath
                                as SplineHeadingPath
                            ).spline
                    )
                )
            }

            @Test
            fun testLinearSplineHeading() {
                val posPath = PositionPathBuilder(
                    Position2(0.0, 0.0),
                    Rotation2.exp(0.0)
                )
                    .splineTo(
                        Position2(15.0, 15.0),
                        Rotation2.exp(PI),
                    )
                    .build()

                val posePath = PosePathBuilder(posPath, Rotation2.exp(PI))
                    .linearUntil(posPath.length / 2, Rotation2.exp(PI / 2))
                    .splineUntilEnd(Rotation2.exp(-PI / 3))

                saveChart("poseBuilder/linearSpline", chartPosePathHeading(posePath))
            }

            @Test
            fun testSplineLinearHeading() {
                val posPath = PositionPathBuilder(
                    Position2(0.0, 0.0),
                    Rotation2.exp(0.0)
                )
                    .splineTo(
                        Position2(15.0, 15.0),
                        Rotation2.exp(PI),
                    )
                    .build()

                val posePath = PosePathBuilder(posPath, Rotation2.exp(PI))
                    .splineUntil(posPath.length / 2, Rotation2.exp(-PI / 3))
                    .linearUntilEnd(Rotation2.exp(PI / 2))

                saveChart("poseBuilder/splineLinear", chartPosePathHeading(posePath))
            }

            @Test
            fun testSplineSplineHeading() {
                val posPath = PositionPathBuilder(
                    Position2(0.0, 0.0),
                    Rotation2.exp(0.0)
                )
                    .splineTo(
                        Position2(15.0, 15.0),
                        Rotation2.exp(PI),
                    )
                    .build()

                val posePath = PosePathBuilder(posPath, Rotation2.exp(PI))
                    .splineUntil(posPath.length / 2, Rotation2.exp(-PI / 3))
                    .splineUntilEnd(Rotation2.exp(PI / 2))

                saveChart("poseBuilder/splineSpline", chartPosePathHeading(posePath))
            }

            @Test
            fun testComplex() {
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
                    .tangentUntil((posPath.offsets[0] + posPath.offsets[1]) / 2)
                    .splineUntilEnd(Rotation2.exp(-PI / 3))

                saveChart("posPathBuilder", chartPosPath(posPath))
                saveChart("posePathBuilder", chartPosePath(posePath))
            }

            @Test
            fun testConstantLinear() {
                val posPath = PositionPathBuilder(
                    Position2(0.0, 0.0),
                    Rotation2.exp(0.0)
                )
                    .splineTo(
                        Position2(15.0, 15.0),
                        Rotation2.exp(PI),
                    )
                    .build()

                assertFails {
                    PosePathBuilder(posPath, Rotation2.exp(0.0))
                        .constantUntil(posPath.length / 2)
                        .linearUntilEnd(Rotation2.exp(PI / 2))
                }
            }

            fun nextRot() = Rotation2.exp(2 * PI * Random.Default.nextDouble())

            fun appendSafe(disp: Double, b: SafePosePathBuilder): PosePath {
                val r = Random.Default
                val x = r.nextDouble()
                return if (r.nextDouble() < 0.1 || disp >= 25.0) {
                    when {
                        x < 0.25 -> b.tangentUntilEnd()
                        x < 0.5 -> b.constantUntilEnd()
                        x < 0.75 -> b.linearUntilEnd(nextRot())
                        else -> b.splineUntilEnd(nextRot())
                    }
                } else {
                    if (x < 0.25) {
                        appendSafe(disp + 1.0, b.splineUntil(disp, nextRot()))
                    } else {
                        appendRestricted(
                            disp + 1.0,
                            when {
                                x < 0.5 -> b.tangentUntil(disp)
                                x < 0.75 -> b.constantUntil(disp)
                                else -> b.linearUntil(disp, nextRot())
                            }
                        )
                    }
                }
            }

            fun appendRestricted(disp: Double, b: RestrictedPosePathBuilder) =
                if (Random.Default.nextDouble() < 0.1 || disp >= 25.0) {
                    b.splineUntilEnd(nextRot())
                } else {
                    appendSafe(disp + 1.0, b.splineUntil(disp, nextRot()))
                }

            @Test
            fun testSafePathBuilder() {
                repeat(100) {
                    val posPath = PositionPathBuilder(
                        Position2(0.0, 0.0),
                        Rotation2.exp(0.0),
                    )
                        .forward(25.0)
                        .build()

                    appendSafe(
                        1.0,
                        SafePosePathBuilder(
                            posPath,
                            Rotation2.exp(0.0)
                        )
                    )
                }
            }
        }
        