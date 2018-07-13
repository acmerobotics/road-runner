package com.acmerobotics.splinelib

import com.acmerobotics.splinelib.trajectory.DriveConstraints
import com.acmerobotics.splinelib.trajectory.Trajectory
import com.acmerobotics.splinelib.trajectory.TrajectoryBuilder
import java.awt.*
import java.awt.geom.AffineTransform
import java.awt.geom.Point2D
import javax.imageio.ImageIO
import javax.swing.JPanel
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.roundToInt


class FieldPanel : JPanel() {
    companion object {
        const val RESOLUTION = 1000
        const val EPSILON = 1e-6
    }

    var poses = listOf<Pose2d>()
    var trajectory = Trajectory()

    init {
        preferredSize = Dimension(500, 500)
    }

    fun updatePoses(poses: List<Pose2d>) {
        this.poses = poses

        // update the trajectory
        trajectory = if (poses.isEmpty()) {
            Trajectory()
        } else {
            val builder = TrajectoryBuilder(poses.first(), DriveConstraints(10.0, 10.0, 10.0, 10.0, 10.0))
            for (i in 1 until poses.size) {
                val startPose = poses[i-1]
                val endPose = poses[i]
                if (abs(startPose.x - endPose.x) < EPSILON && abs(startPose.y - endPose.y) < EPSILON) {
                    // this is probably a turn
                    builder.turnTo(endPose.heading)
                } else {
                    builder.beginComposite()
                    val diff = endPose - startPose
                    val cosAngle = (Math.cos(endPose.heading) * diff.x + Math.sin(endPose.heading) * diff.y) / diff.pos().norm()

                    builder.setReversed(cosAngle < 0)

                    if (abs(startPose.heading - endPose.heading)  < EPSILON && abs(1 - abs(cosAngle)) < EPSILON) {
                        // this is probably a line
                        builder.lineTo(endPose.pos())
                    } else {
                        // this is probably a spline
                        builder.splineTo(endPose)
                    }
                }
            }
            builder.build()
        }

        repaint()
    }

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

        val transform = AffineTransform()
        transform.translate(width / 2.0, height / 2.0)
        transform.scale(fieldSize / 144.0, fieldSize / 144.0)
        transform.rotate(Math.PI / 2)
        transform.scale(-1.0, 1.0)

        // draw field
        val fieldImage = ImageIO.read(javaClass.getResource("/transparent_field.png"));
        g2d.drawImage(fieldImage, offsetX.roundToInt(), offsetY.roundToInt(), fieldSize, fieldSize, null)

        g2d.color = Color(76, 175, 80)

        // draw poses
        for (pose in poses) {
            val point = Point2D.Double()
            transform.transform(Point2D.Double(pose.x, pose.y), point)
            g2d.fillArc(point.x.roundToInt() - 5, point.y.roundToInt() - 5, 10, 10, 0, 360)
        }

        // draw trajectory
        g2d.stroke = BasicStroke(3F)

        if (trajectory.duration() == 0.0) {
            return
        }

        val displacements = (0..RESOLUTION).map { it / RESOLUTION.toDouble() * trajectory.duration() }
        for (i in 1..RESOLUTION) {
            val firstPose = trajectory[displacements[i-1]]
            val secondPose = trajectory[displacements[i]]
            val firstPoint = Point2D.Double()
            val secondPoint = Point2D.Double()
            transform.transform(Point2D.Double(firstPose.x, firstPose.y), firstPoint)
            transform.transform(Point2D.Double(secondPose.x, secondPose.y), secondPoint)
            g2d.drawLine(firstPoint.x.roundToInt(), firstPoint.y.roundToInt(), secondPoint.x.roundToInt(), secondPoint.y.roundToInt())
        }
    }
}