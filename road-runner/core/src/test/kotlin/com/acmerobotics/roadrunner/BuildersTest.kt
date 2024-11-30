package com.acmerobotics.roadrunner

import org.knowm.xchart.QuickChart
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.theme.MatlabTheme
import kotlin.math.PI
import kotlin.random.Random
import kotlin.test.Test
import kotlin.test.assertEquals

fun chartSpline(q: QuinticSpline1d): XYChart {
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

        fun chartSplineExpLog(q: QuinticSpline1d): XYChart {
            val ts = range(0.0, 1.0, 1000)
            val xs = ts.map { Rotation2dDual.exp(q[it, 3]) + 0.3 }

            return QuickChart.getChart(
                "Spline", "t", "",
                arrayOf("x", "dx", "d2x"),
                ts.toDoubleArray(),
                arrayOf(
                    xs.map { it.value().log() }.toDoubleArray(),
                    xs.map { it.velocity()[0] }.toDoubleArray(),
                    xs.map { it.velocity()[1] }.toDoubleArray(),
                )
            ).also {
                it.styler.theme = MatlabTheme()
            }
        }

        fun <Param> chartPosPath(posPath: PositionPath<Param>): XYChart {
            val params = range(-1.0, posPath.length() + 1.0, 1000)
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
            val params = range(-1.0, posePath.length() + 1.0, 1000)
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
                    poses.map { it.position.x[1] }.toDoubleArray(),
                    poses.map { it.position.y[1] }.toDoubleArray(),
                    poses.map { it.heading.velocity()[0] }.toDoubleArray(),
                    poses.map { it.position.x[2] }.toDoubleArray(),
                    poses.map { it.position.y[2] }.toDoubleArray(),
                    poses.map { it.heading.velocity()[1] }.toDoubleArray(),
                )
            )
        }

        fun chartPosePathHeading(posePath: PosePath): XYChart {
            val params = range(-1.0, posePath.length() + 1.0, 1000)
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
                    poses.map { it.heading.value().log() }.toDoubleArray(),
                    poses.map { it.heading.velocity()[0] }.toDoubleArray(),
                    poses.map { it.heading.velocity()[1] }.toDoubleArray(),
                )
            )
        }

        fun chartPosePathXY(posePath: PosePath): XYChart {
            val params = range(-1.0, posePath.length() + 1.0, 1000)
            val positions = params.map { posePath[it, 1].position.value() }

            return QuickChart.getChart(
                "Path", "x", "y", "Path",
                positions.map { it.x }.toDoubleArray(),
                positions.map { it.y }.toDoubleArray(),
            )
        }

        class BuildersTest {
            @Test
            fun testLineToX() {
                val r = Random.Default
                repeat(100) {
                    val beginPos = Vector2d(r.nextDouble(), r.nextDouble())
                    val beginTangent = Rotation2d.exp(r.nextDouble())

                    val posX = r.nextDouble()

                    val posPath = PositionPathSeqBuilder(beginPos, beginTangent, 1e-6)
                        .lineToX(posX)
                        .build()
                        .first()

                    assertEquals(beginPos.x, posPath.begin(1).value().x, 1e-6)
                    assertEquals(beginPos.y, posPath.begin(1).value().y, 1e-6)

                    assertEquals(posX, posPath.end(1).value().x, 1e-6)
                }
            }

            @Test
            fun testSplineTo() {
                val r = Random.Default
                repeat(100) {
                    val beginPos = Vector2d(r.nextDouble(), r.nextDouble())
                    val beginTangent = Rotation2d.exp(r.nextDouble())
                    val endPos = Vector2d(r.nextDouble(), r.nextDouble())
                    val endTangent = Rotation2d.exp(r.nextDouble())

                    val posPath = PositionPathSeqBuilder(beginPos, beginTangent, 1e-6)
                        .splineTo(endPos, endTangent)
                        .build()
                        .first()

                    assertEquals(beginPos.x, posPath.begin(1).value().x, 1e-6)
                    assertEquals(beginPos.y, posPath.begin(1).value().y, 1e-6)
                    assertEquals(0.0, beginTangent - posPath.begin(2).drop(1).angleCast().value(), 1e-6)

                    assertEquals(endPos.x, posPath.end(1).value().x, 1e-6)
                    assertEquals(endPos.y, posPath.end(1).value().y, 1e-6)
                    assertEquals(0.0, endTangent - posPath.end(2).drop(1).angleCast().value(), 1e-6)
                }
            }

            @Test
            fun testTangentHeading() {
                val posPath = PositionPathSeqBuilder(
                    Vector2d(0.0, 0.0),
                    Rotation2d.exp(0.0),
                    1e-6
                )
                    .splineTo(
                        Vector2d(15.0, 15.0),
                        Rotation2d.exp(PI),
                    )
                    .build()
                    .first()

                val posePath = PosePathSeqBuilder(posPath, Rotation2d.exp(PI))
                    .tangentUntilEnd()
                    .first()

                saveChart("poseBuilder/tangent", chartPosePathHeading(posePath))
            }

            @Test
            fun testSplineHeading() {
                val posPathPre = PositionPathSeqBuilder(
                    Vector2d(0.0, 0.0),
                    Rotation2d.exp(0.0),
                    1e-6,
                )
                    .splineTo(
                        Vector2d(15.0, 15.0),
                        Rotation2d.exp(PI),
                    )
//            .lineTo(
//                Position2(100.0, 0.0)
//            )
                    .build()
                    .first()

                val posPath = PositionPathSeqBuilder(
                    Vector2d(0.0, 0.0),
                    Rotation2d.exp(0.0),
                    1e-6,
                )
                    .splineTo(
                        Vector2d(15.0, 15.0) / posPathPre.length,
                        Rotation2d.exp(PI),
                    )
                    .build()
                    .first()

                val posePath = PosePathSeqBuilder(posPath, Rotation2d.exp(PI))
                    .splineUntilEnd(Rotation2d.exp(-PI / 3))
                    .first()

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
                val posPath = PositionPathSeqBuilder(
                    Vector2d(0.0, 0.0),
                    Rotation2d.exp(0.0),
                    1e-6,
                )
                    .splineTo(
                        Vector2d(15.0, 15.0),
                        Rotation2d.exp(PI),
                    )
                    .build()
                    .first()

                val posePath = PosePathSeqBuilder(posPath, Rotation2d.exp(PI))
                    .linearUntil(posPath.length / 2, Rotation2d.exp(PI / 2))
                    .splineUntilEnd(Rotation2d.exp(-PI / 3))
                    .first()

                saveChart("poseBuilder/linearSpline", chartPosePathHeading(posePath))
            }

            @Test
            fun testSplineLinearHeading() {
                val posPath = PositionPathSeqBuilder(
                    Vector2d(0.0, 0.0),
                    Rotation2d.exp(0.0),
                    1e-6,
                )
                    .splineTo(
                        Vector2d(15.0, 15.0),
                        Rotation2d.exp(PI),
                    )
                    .build()
                    .first()

                val posePath = PosePathSeqBuilder(posPath, Rotation2d.exp(PI))
                    .splineUntil(posPath.length / 2, Rotation2d.exp(-PI / 3))
                    .linearUntilEnd(Rotation2d.exp(PI / 2))
                    .first()

                saveChart("poseBuilder/splineLinear", chartPosePathHeading(posePath))
            }

            @Test
            fun testSplineSplineHeading() {
                val posPath = PositionPathSeqBuilder(
                    Vector2d(0.0, 0.0),
                    Rotation2d.exp(0.0),
                    1e-6,
                )
                    .splineTo(
                        Vector2d(15.0, 15.0),
                        Rotation2d.exp(PI),
                    )
                    .build()
                    .first()

                val posePath = PosePathSeqBuilder(posPath, Rotation2d.exp(PI))
                    .splineUntil(posPath.length / 2, Rotation2d.exp(-PI / 3))
                    .splineUntilEnd(Rotation2d.exp(PI / 2))
                    .first()

                saveChart("poseBuilder/splineSpline", chartPosePathHeading(posePath))
            }

            @Test
            fun testComplex() {
                val posPath = PositionPathSeqBuilder(
                    Vector2d(0.0, 0.0),
                    Rotation2d.exp(0.0),
                    1e-6,
                )
                    .splineTo(
                        Vector2d(15.0, 15.0),
                        Rotation2d.exp(PI),
                    )
                    .splineTo(
                        Vector2d(5.0, 35.0),
                        Rotation2d.exp(PI / 3),
                    )
                    .build()
                    .first()

                val posePath = PosePathSeqBuilder(posPath, Rotation2d.exp(0.0))
                    .tangentUntil((posPath.offsets[0] + posPath.offsets[1]) / 2)
                    .splineUntilEnd(Rotation2d.exp(-PI / 3))
                    .first()

                saveChart("posPathBuilder", chartPosPath(posPath))
                saveChart("posePathBuilder", chartPosePath(posePath))
            }

            @Test
            fun testPathBuilderLineToX() {
                val posePath = PathBuilder(
                    Pose2d(
                        Vector2d(0.0, 0.0),
                        Rotation2d.exp(0.0),
                    ),
                    1e-6,
                )
                    .lineToXLinearHeading(10.0, PI / 2)
                    .lineToXSplineHeading(20.0, -PI / 2)
                    .build()
                    .first()

                saveChart("pathBuilder/lineToX", chartPosePath(posePath))
            }

            @Test
            fun testIssue82() {
                val traj = TrajectoryBuilder(
                    TEST_TRAJECTORY_BUILDER_PARAMS,
                    Pose2d(0.0, 0.0, 0.0),
                    0.0,
                    MinVelConstraint(
                        listOf(
                            MecanumKinematics(7.0, 1.0).WheelVelConstraint(10.0),
                            AngularVelConstraint(PI / 4)
                        )
                    ),
                    ProfileAccelConstraint(-10.0, 15.0),
                )
                    .splineTo(Vector2d(20.0, -20.0), -PI / 2)
                    .build()
                    .first()

                saveChart("trajBuilder/issue82", chartPosePath(traj.path))
                saveChart("trajBuilder/issue82XY", chartPosePathXY(traj.path))
                saveChart("trajBuilder/issue82Profile", chartTimeProfile(TimeProfile(traj.profile.baseProfile)))
            }

            // https://github.com/acmerobotics/road-runner/issues/97
            @Test
            fun testIssue97() {
                val begin = Pose2d(-12.0, -62.0, Math.toRadians(270.0))
                val end = Pose2d(-12.0, -48.0, Math.toRadians(270.0))

                TrajectoryBuilder(
                    // fails with the default test params
                    // TEST_TRAJECTORY_BUILDER_PARAMS,
                    TEST_TRAJECTORY_BUILDER_PARAMS.copy(
                        profileParams = TEST_PROFILE_PARAMS.copy(
                            angSamplingEps = 1e-2
                        )
                    ),
                    begin,
                    0.0,
                    MinVelConstraint(
                        listOf(
                            MecanumKinematics(7.0, 1.0).WheelVelConstraint(10.0),
                            AngularVelConstraint(PI / 4)
                        )
                    ),
                    ProfileAccelConstraint(-10.0, 15.0),
                )
                    .splineTo(end.position, end.heading)
                    .build()
            }

            @Test
            fun testConstantLinear() {
                val posPath = PositionPathSeqBuilder(
                    Vector2d(0.0, 0.0),
                    Rotation2d.exp(0.0),
                    1e-6,
                )
                    .splineTo(
                        Vector2d(15.0, 15.0),
                        Rotation2d.exp(PI),
                    )
                    .build()
                    .first()

                assertEquals(
                    2,
                    PosePathSeqBuilder(posPath, Rotation2d.exp(0.0))
                        .constantUntil(posPath.length / 2)
                        .linearUntilEnd(Rotation2d.exp(PI / 2))
                        .size
                )
            }

            fun nextRot() = Rotation2d.exp(2 * PI * Random.Default.nextDouble())

            fun appendSafe(disp: Double, b: SafePosePathBuilder): List<PosePath> {
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
                    val posPath = PositionPathSeqBuilder(
                        Vector2d(0.0, 0.0),
                        Rotation2d.exp(0.0),
                        1e-6,
                    )
                        .lineToX(25.0)
                        .build()
                        .first()

                    appendSafe(
                        1.0,
                        SafePosePathBuilder(
                            posPath,
                            Rotation2d.exp(0.0)
                        )
                    )
                }
            }

            @Test
            fun testBackwardLine() {
                PositionPathSeqBuilder(
                    Vector2d(0.0, 0.0),
                    Rotation2d.exp(0.0),
                    1e-6,
                )
                    .lineToX(-10.0)
                    .build()
            }

            @Test
            fun testBackwardLine2() {
                PositionPathSeqBuilder(
                    Vector2d(0.0, 0.0),
                    Rotation2d.exp(0.0),
                    1e-6,
                )
                    .lineToX(-10.0)
                    .build()
            }

            @Test
            fun testLineSpline() {
                TrajectoryBuilder(
                    TEST_TRAJECTORY_BUILDER_PARAMS,
                    Pose2d(0.0, 0.0, 0.0), 0.0,
                    TranslationalVelConstraint(40.0),
                    ProfileAccelConstraint(-30.0, 50.0),
                )
                    .lineToX(20.0)
                    .splineTo(Vector2d(40.0, 55.0), PI / 2)
                    .build()
            }

            @Test
            fun testTrivialLineToX() {
                TrajectoryBuilder(
                    TEST_TRAJECTORY_BUILDER_PARAMS,
                    Pose2d(0.0, 0.0, 0.0), 0.0,
                    TranslationalVelConstraint(40.0),
                    ProfileAccelConstraint(-30.0, 50.0),
                )
                    .lineToX(0.0)
                    .build()
            }

            @Test
            fun testTrivialLineToX2() {
                TrajectoryBuilder(
                    TEST_TRAJECTORY_BUILDER_PARAMS,
                    Pose2d(0.0, 0.0, 0.0), 0.0,
                    TranslationalVelConstraint(40.0),
                    ProfileAccelConstraint(-30.0, 50.0),
                )
                    .lineToX(0.0)
                    .lineToX(10.0)
                    .build()
            }

            @Test
            fun testTrivialSpline() {
                TrajectoryBuilder(
                    TEST_TRAJECTORY_BUILDER_PARAMS,
                    Pose2d(0.0, 0.0, 0.0), 0.0,
                    TranslationalVelConstraint(40.0),
                    ProfileAccelConstraint(-30.0, 50.0),
                )
                    .splineTo(Vector2d(0.0, 0.0), 0.0)
                    .build()
            }
        }
        