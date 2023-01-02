package com.acmerobotics.roadrunner

import java.awt.BasicStroke
import java.awt.Color
import java.awt.Graphics2D
import java.awt.RenderingHints
import java.awt.geom.Ellipse2D
import java.awt.geom.Path2D
import java.awt.image.BufferedImage
import java.io.File
import java.lang.RuntimeException
import java.nio.file.Path
import javax.imageio.ImageIO
import kotlin.io.path.createTempDirectory
import kotlin.math.PI
import kotlin.math.ceil
import kotlin.math.roundToInt

fun withTempDir(f: (Path) -> Unit) {
    val dir = createTempDirectory()
    try {
        f(dir)
    } finally {
        dir.toFile().deleteRecursively()
    }
}

class FieldGraphics(
    val g: Graphics2D,
    halfSize: Int,
) {
    init {
        // from https://stackoverflow.com/questions/1094539/how-to-draw-a-decent-looking-circle-in-java
        g.setRenderingHints(
            RenderingHints(
                RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON
            )
        )

        g.translate(halfSize, halfSize)
        g.rotate(-PI / 2)
        val scale = halfSize / 84.0
        g.scale(scale, -scale)
    }

    fun setStrokeWidth(w: Double) {
        g.stroke = BasicStroke(w.toFloat())
    }

    fun setColor(c: Color) {
        g.color = c
    }

    fun translate(v: Vector2d) {
        g.translate(v.x, v.y)
    }

    fun scale(v: Vector2d) {
        g.scale(v.x, v.y)
    }

    fun strokeCircle(v: Vector2d, radius: Double) {
        g.draw(
            Ellipse2D.Double(
                v.x - radius,
                v.y - radius,
                2 * radius, 2 * radius,
            )
        )
    }

    fun fillCircle(v: Vector2d, radius: Double) {
        // from https://stackoverflow.com/questions/9650000/how-to-draw-a-circle-positioning-it-at-a-double-value-instead-of-a-int
        g.fill(
            Ellipse2D.Double(
                v.x - radius,
                v.y - radius,
                2 * radius, 2 * radius,
            )
        )
    }

    fun strokeLine(v: Vector2d, w: Vector2d) = strokeLine(listOf(v, w))

    fun strokeLine(vs: List<Vector2d>) {
        val path = Path2D.Double()
        path.moveTo(vs[0].x, vs[0].y)
        for (v in vs.drop(1)) {
            path.lineTo(v.x, v.y)
        }
        g.draw(path)
    }
}

fun videoBuilder(beginPose: Pose2d = Pose2d(0.0, 0.0, 0.0)) =
    TrajectoryActionBuilder(
        { TurnAction(it) },
        { TrajectoryAction(it) },
        beginPose,
        1e-6,
        TurnConstraints(PI / 2, -PI / 2, PI / 2),
        MinVelConstraint(
            listOf(
                TranslationalVelConstraint(50.0),
                AngularVelConstraint(PI / 2),
            )
        ),
        ProfileAccelConstraint(-40.0, 40.0),
        0.25,
    )

fun writeVideo(f: File, a: Action, fps: Int = 5) {
    fun actionTimeline(a: Action): Pair<Double, List<Pair<Double, Action>>> {
        val timeline = mutableListOf<Pair<Double, Action>>()

        fun aux(t0: Double, a: Action): Double {
            when (a) {
                is SequentialAction -> {
                    var t = t0
                    for (a2 in a.initialActions) {
                        t = aux(t, a2)
                    }
                    return t
                }

                is ParallelAction -> {
                    return a.initialActions.maxOf { a2 ->
                        aux(t0, a2)
                    }
                }

                is TrajectoryAction -> {
                    timeline.add(Pair(t0, a))
                    return t0 + a.t.profile.duration
                }

                is TurnAction -> {
                    timeline.add(Pair(t0, a))
                    return t0 + a.t.profile.duration
                }

                is SleepAction -> {
                    return t0 + a.dt
                }

                is LabelAction -> {
                    timeline.add(Pair(t0, a))
                    return t0
                }

                else -> {
                    throw RuntimeException()
                }
            }
        }

        val dt = aux(0.0, a)

        timeline.sortBy { it.first }

        return Pair(dt, timeline)
    }

    println(a)

    val diskIm = ImageIO.read(File("actions/src/test/resources/background-dark.png"))
    require(diskIm.width == diskIm.height)
    require(diskIm.width % 2 == 0)
    val halfSize = diskIm.width / 2

    val timelineHeight = 48.0

    val bgIm = BufferedImage(
        diskIm.width,
        // NOTE: Video encoder requires the height to be even
        diskIm.height + (timelineHeight * halfSize / 84.0).roundToInt() + 1,
        BufferedImage.TYPE_3BYTE_BGR
    )

    val (dt, timeline) = actionTimeline(a)

    val profileWidth = 2.0 * (84.0 - 4.0)
    val profileHeight = timelineHeight - 2 * 4.0
    val timeGap = dt / (profileWidth / 2.0)

    val sampledTrajs = mutableListOf<List<Pair<Double, Double>>>()
    val sampledTurns = mutableListOf<List<Pair<Double, Double>>>()
    val labelTimes = mutableListOf<Double>()
    for ((offset, a) in timeline) {
        when (a) {
            is TrajectoryAction -> {
                val nsamples = ceil(a.t.profile.duration / timeGap).toInt()
                sampledTrajs.add(
                    range(0.0, a.t.profile.duration, nsamples).map { t ->
                        Pair(offset + t, a.t.profile[t][1])
                    }
                )
            }
            is TurnAction -> {
                val nsamples = ceil(a.t.profile.duration / timeGap).toInt()
                sampledTurns.add(
                    range(0.0, a.t.profile.duration, nsamples).map { t ->
                        Pair(offset + t, a.t.profile[t][1])
                    }
                )
            }
            is LabelAction -> {
                labelTimes.add(offset)
            }
        }
    }

    run {
        // draw the axes and preview
        val gawt = bgIm.createGraphics()
        gawt.drawImage(diskIm, 0, 0, null)
        val g = FieldGraphics(gawt, halfSize)

        g.setStrokeWidth(1.0)

        val turns = mutableListOf<TimeTurn>()

        fun drawPreview(a: Action) {
            when (a) {
                is SequentialAction -> a.initialActions.forEach(::drawPreview)
                is ParallelAction -> a.initialActions.forEach(::drawPreview)
                is TrajectoryAction -> {
                    g.setColor(Color(0x7A4CAF50, true))
                    g.strokeLine(
                        range(0.0, a.t.path.length(), ceil(a.t.path.length() / 2.0).toInt())
                            .map { s -> a.t.path[s, 1].value().trans }
                    )
                }
                is TurnAction -> {
                    turns.add(a.t)
                }
            }
        }

        drawPreview(a)

        for (turn in turns) {
            g.setColor(Color(0x7A7C4DFF, true))
            g.fillCircle(turn.beginPose.trans, 2.0)
        }

        // draw the profile
        // place the origin in the bottom left with +axes going up and to the right
        g.translate(Vector2d(-84.0 - timelineHeight, 84.0))
        g.scale(Vector2d(1.0, -1.0))
        g.translate(Vector2d(4.0, 4.0))

        val maxTrajV = lazy { sampledTrajs.maxOf { it.maxOf { it.second } } }
        g.setColor(Color(0x7A4CAF50, true))
        for (sampledTraj in sampledTrajs) {
            g.strokeLine(
                sampledTraj.map {
                    Vector2d(it.second / maxTrajV.value * profileHeight, it.first / dt * profileWidth)
                }
            )
        }

        val maxTurnV = lazy { sampledTurns.maxOf { it.maxOf { it.second } } }
        g.setColor(Color(0x7A7C4DFF, true))
        for (sampledTurn in sampledTurns) {
            g.strokeLine(
                sampledTurn.map {
                    Vector2d(it.second / maxTurnV.value * profileHeight, it.first / dt * profileWidth)
                }
            )
        }

        g.setColor(Color(0x7AD9B124, true))
        for (labelTime in labelTimes) {
            g.strokeLine(
                Vector2d(0.0, labelTime / dt * profileWidth),
                Vector2d(profileHeight, labelTime / dt * profileWidth)
            )
        }
    }

    // from https://stackoverflow.com/questions/3514158/how-do-you-clone-a-bufferedimage
    fun newImage() = BufferedImage(
        bgIm.colorModel,
        bgIm.copyData(null),
        bgIm.colorModel.isAlphaPremultiplied,
        null
    )

    withTempDir { dir ->
        val times = (0..ceil(fps * (dt + 2.0)).toInt()).map { it / fps.toDouble() - 1.0 }
        for ((i, time) in times.withIndex()) {
            val im = newImage()
            val g = FieldGraphics(im.createGraphics(), halfSize)

            val (beginTime, a) = timeline
                .filter { (beginTime, a) -> beginTime <= time && (a is TrajectoryAction || a is TurnAction) }
                .maxByOrNull { (beginTime, _) -> beginTime } ?: timeline.first()
            val pose = when (a) {
                is TrajectoryAction -> a.t[time - beginTime].value()
                is TurnAction -> a.t[time - beginTime].value()
                else -> {
                    throw RuntimeException()
                }
            }
            g.setStrokeWidth(1.0)
            g.setColor(Color(0x4CAF50, false))
            g.strokeCircle(pose.trans, 9.0)
            g.strokeLine(pose.trans, pose.trans + pose.rot.vec() * 9.0)

            g.translate(Vector2d(-84.0 - timelineHeight, 84.0))
            g.scale(Vector2d(1.0, -1.0))
            g.translate(Vector2d(4.0, 4.0))

            g.strokeLine(
                Vector2d(0.0, clamp(time, 0.0, dt) / dt * profileWidth),
                Vector2d(profileHeight, clamp(time, 0.0, dt) / dt * profileWidth)
            )

            // This takes most of the time.
            fun write() {
                ImageIO.write(
                    im, "png",
                    File(
                        dir.toFile(),
                        "image${i.toString().padStart(4, '0')}.png"
                    )
                )
            }

            write()
        }

        // from https://stackoverflow.com/questions/24961127/how-to-create-a-video-from-images-with-ffmpeg
        // ffmpeg -framerate 30 -pattern_type glob -i '*.png' \
        //   -c:v libx264 -pix_fmt yuv420p out.mp4
        val p = ProcessBuilder()
            .inheritIO()
            .command(
                "ffmpeg", "-y", "-framerate", fps.toString(),
                "-pattern_type", "glob", "-i", File(dir.toFile(), "*.png").toString(),
                "-c:v", "libx264", "-pix_fmt", "yuv420p", f.absolutePath,
            )
            .start()
        p.waitFor()
    }
}
