package com.acmerobotics.roadrunner.gui

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig
import com.acmerobotics.roadrunner.util.NanoClock
import java.awt.*
import java.awt.event.MouseEvent
import java.awt.event.MouseListener
import java.awt.geom.*
import javax.imageio.ImageIO
import javax.swing.JPanel
import javax.swing.Timer
import kotlin.math.min
import kotlin.math.roundToInt

private const val SPATIAL_RESOLUTION = 0.25  // in
private const val TEMPORAL_RESOLUTION = 0.025  // sec
private val FIELD_IMAGE = ImageIO.read(FieldPanel::class.java.getResource("/field.png"))

fun Vector2d.awt() = Point2D.Double(x, y)

fun Path2D.Double.moveTo(point: Point2D.Double) {
    moveTo(point.x, point.y)
}

fun Path2D.Double.lineTo(point: Point2D.Double) {
    lineTo(point.x, point.y)
}

fun circle(center: Vector2d, radius: Double) = Ellipse2D.Double(center.x - radius / 2, center.y - radius / 2, radius, radius)

/**
 * Panel displaying an image of the field with the trajectory/path superimposed.
 */
class FieldPanel : JPanel() {

    private val fieldTransform = AffineTransform()
    private val robotTransform = AffineTransform()

    private var robotPath: Path2D.Double = Path2D.Double()
    private var robotRect = Rectangle2D.Double()

    private val updateLock = Object()

    private var knots = listOf<Vector2d>()
    private var trajectory: Trajectory? = null
    private var path: Path2D.Double = Path2D.Double()
    private var area: Area = Area()

    private var timer: Timer? = null
    private var startTime: Double = 0.0
    private val clock = NanoClock.system()

    init {
        preferredSize = Dimension(500, 500)

        addMouseListener(object : MouseListener {
            override fun mouseReleased(e: MouseEvent?) {

            }

            override fun mouseEntered(e: MouseEvent?) {
                startAnimation()
            }

            override fun mouseClicked(e: MouseEvent?) {

            }

            override fun mouseExited(e: MouseEvent?) {
                stopAnimation()
            }

            override fun mousePressed(e: MouseEvent?) {

            }
        })
    }

    fun updateTrajectoryAndConfig(trajectory: Trajectory, config: TrajectoryConfig, groupConfig: TrajectoryGroupConfig) {
        val newPath = Path2D.Double()
        val newArea = Area()

        // compute robot path and rect
        val length = groupConfig.robotLength
        val width = groupConfig.robotWidth
        robotPath = Path2D.Double()
        robotPath.moveTo(length / 2, 0.0)
        robotPath.lineTo(0.0, 0.0)
        robotPath.lineTo(length / 2, 0.0)
        robotPath.lineTo(length / 2, width / 2)
        robotPath.lineTo(-length / 2, width / 2)
        robotPath.lineTo(-length / 2, -width / 2)
        robotPath.lineTo(length / 2, -width / 2)
        robotPath.closePath()

        robotRect = Rectangle2D.Double(-length / 2, -width / 2, length, width)

        // compute path samples
        val displacementSamples = (trajectory.path.length() / SPATIAL_RESOLUTION).roundToInt()
        val displacements = (0..displacementSamples).map {
            it / displacementSamples.toDouble() * trajectory.path.length()
        }

        // compute the path segments and area swept by the robot
        val poses = displacements.map { trajectory.path[it] }
        newPath.moveTo(poses.first().vec().awt())
        for (pose in poses.drop(1)) {
            if (Thread.currentThread().isInterrupted) return

            newPath.lineTo(pose.vec().awt())
        }

        // compute time samples
        val timeSamples = (trajectory.duration() / TEMPORAL_RESOLUTION).roundToInt()
        val times = (0..timeSamples).map {
            it / timeSamples.toDouble() * trajectory.duration()
        }

        // TODO: is this procedure quadratic?
        // is it better to divide and conquer (at the cost of additional memory)?
        for (time in times) {
            if (Thread.currentThread().isInterrupted) return

            val pose = trajectory[time]
            robotTransform.setToTranslation(pose.x, pose.y)
            robotTransform.rotate(pose.heading)
            newArea.add(Area(robotTransform.createTransformedShape(robotRect)))
        }

        synchronized(updateLock) {
            this.knots = listOf(config.startPose.vec()) + config.steps.map { it.pose.vec() }
            this.trajectory = trajectory
            this.path = newPath
            this.area = newArea
        }

        startTime = clock.seconds()

        repaint()
    }

    private fun startAnimation() {
        timer = Timer(35) {
            repaint()
        }
        timer?.start()
        startTime = clock.seconds()
    }

    private fun stopAnimation() {
        timer?.stop()
        timer = null
        repaint()
    }

    @Suppress("LongMethod")
    override fun paintComponent(g: Graphics?) {
        super.paintComponent(g)

        val g2d = g as Graphics2D

        // antialiasing
        g2d.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON)
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)
        g2d.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY)

        // transform coordinate frame
        val fieldSize = min(width, height)
        val offsetX = ((width - fieldSize) / 2.0).roundToInt()
        val offsetY = ((height - fieldSize) / 2.0).roundToInt()

        fieldTransform.setToTranslation(width / 2.0, height / 2.0)
        fieldTransform.scale(fieldSize / 144.0, fieldSize / 144.0)
        fieldTransform.rotate(Math.PI / 2)
        fieldTransform.scale(-1.0, 1.0)

        // draw field
        g2d.drawImage(FIELD_IMAGE, offsetX, offsetY, fieldSize, fieldSize, null)

        g2d.color = SERIES_COLORS[2]
        g2d.paint = SERIES_COLORS[2]

        synchronized(updateLock) {
            // draw poses
            for (knot in knots) {
                g2d.fill(fieldTransform.createTransformedShape(circle(knot, 3.0)))
            }

            if (trajectory == null) return

            // draw path
            g2d.stroke = BasicStroke(3F)
            g2d.draw(fieldTransform.createTransformedShape(path))

            // draw trajectory
            if (timer != null) {
                val elapsedTime = clock.seconds() - startTime
                if (elapsedTime > trajectory?.duration() ?: 0.0) {
                    startTime = clock.seconds()
                    return
                }
                val currentPose = trajectory?.get(elapsedTime) ?: Pose2d()
                g2d.paint = Color.black
                robotTransform.setTransform(fieldTransform)
                robotTransform.translate(currentPose.x, currentPose.y)
                robotTransform.rotate(currentPose.heading)
                g2d.draw(robotTransform.createTransformedShape(robotPath))
                g2d.fill(fieldTransform.createTransformedShape(circle(currentPose.vec(), 3.0)))
            } else {
                g2d.paint = Color(SERIES_COLORS[2].red, SERIES_COLORS[2].green, SERIES_COLORS[2].blue, 120)
                g2d.fill(fieldTransform.createTransformedShape(area))
            }
        }
    }
}
