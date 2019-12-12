package com.acmerobotics.roadrunner.gui

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import java.awt.*
import java.awt.geom.*
import javax.imageio.ImageIO
import javax.swing.JPanel
import kotlin.math.min
import kotlin.math.roundToInt

private const val RESOLUTION = 100
private const val ROBOT_SIZE = 18.0

/**
 * Panel displaying an image of the field with the trajectory/path superimposed.
 */
class FieldPanel : JPanel() {

    private var poses = listOf<Pose2d>()
    private var trajectory: Trajectory? = null

    init {
        preferredSize = Dimension(500, 500)
    }

    fun updateTrajectoryAndPoses(trajectory: Trajectory, poses: List<Pose2d>) {
        this.poses = poses
        this.trajectory = trajectory
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
        val offsetX = (width - fieldSize) / 2.0
        val offsetY = (height - fieldSize) / 2.0

        val fieldTrans = AffineTransform()
        fieldTrans.translate(width / 2.0, height / 2.0)
        fieldTrans.scale(fieldSize / 144.0, fieldSize / 144.0)
        fieldTrans.rotate(Math.PI / 2)
        fieldTrans.scale(-1.0, 1.0)

        // draw field
        val fieldImage = ImageIO.read(javaClass.getResource("/field.png"))
        g2d.drawImage(fieldImage, offsetX.roundToInt(), offsetY.roundToInt(), fieldSize, fieldSize, null)

        g2d.color = SERIES_COLORS[2]

        // draw poses
        val circle = Ellipse2D.Double(0.0, 0.0, 3.0, 3.0)
        for (pose in poses) {
            circle.x = pose.x - circle.width / 2
            circle.y = pose.y - circle.height / 2
            g2d.fill(fieldTrans.createTransformedShape(circle))
        }

        if (trajectory == null) return

        // draw trajectory
        g2d.stroke = BasicStroke(3F)

        if (trajectory?.duration() == 0.0) {
            return
        }

        val displacements = (0..RESOLUTION).map {
            it / RESOLUTION.toDouble() * (trajectory?.duration() ?: 1.0)
        }

        val robotTrans = AffineTransform()
        val robotRect = Rectangle2D.Double(-ROBOT_SIZE / 2, -ROBOT_SIZE / 2, ROBOT_SIZE, ROBOT_SIZE)

        val firstPoint = Point2D.Double()
        val secondPoint = Point2D.Double()
        val line = Line2D.Double()
        for (i in 1..RESOLUTION) {
            val firstPose = trajectory?.get(displacements[i - 1]) ?: Pose2d()
            val secondPose = trajectory?.get(displacements[i]) ?: Pose2d()

            fieldTrans.transform(Point2D.Double(firstPose.x, firstPose.y), firstPoint)
            fieldTrans.transform(Point2D.Double(secondPose.x, secondPose.y), secondPoint)
            line.setLine(firstPoint, secondPoint)

            // draw the path segment
            g2d.draw(line)
        }

        g2d.paint = Color(SERIES_COLORS[2].red, SERIES_COLORS[2].green, SERIES_COLORS[2].blue, 120)
        val area = Area()
        for (i in 0..RESOLUTION) {
            val firstPose = trajectory?.get(displacements[i]) ?: Pose2d()

            // draw the robot rect
            robotTrans.setTransform(fieldTrans)
            robotTrans.translate(firstPose.x, firstPose.y)
            robotTrans.rotate(firstPose.heading)
            area.add(Area(robotTrans.createTransformedShape(robotRect)))
        }
        g2d.fill(area)
    }
}
