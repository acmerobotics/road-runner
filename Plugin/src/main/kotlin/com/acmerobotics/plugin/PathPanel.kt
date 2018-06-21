package com.acmerobotics.plugin

import com.acmerobotics.library.BezierSplineSegment
import com.acmerobotics.library.Waypoint
import java.awt.BasicStroke
import java.awt.Color
import java.awt.Graphics
import java.awt.Graphics2D
import javax.swing.JPanel

class PathPanel: JPanel() {
    override fun paint(g: Graphics?) {
        super.paint(g)

        val g2d = g as Graphics2D
        val splineSegment = BezierSplineSegment.fromWaypoints(
            Waypoint(0.0, 0.0, -5.0, 0.0),
            Waypoint(10.0, 5.0, 1.0, 0.0)
        )

        val (xOffset, yOffset) = if (height > width) {
            Pair(0.0, (height - width) / 2.0)
        } else {
            Pair((width - height) / 2.0, 0.0)
        }

        val fieldSize = Math.min(width, height)
        val scale = fieldSize / 20.0

        g2d.translate(xOffset + fieldSize / 2.0, yOffset + fieldSize / 2.0)
        g2d.stroke = BasicStroke(3.0F)

        val indices = (1..1000).map { it / 1000.0 }
        val points = indices.map { splineSegment[it] * scale }
        val curvatureList = indices.map { splineSegment.curvature(it) }

        val curvatureScale = 1.0 / (curvatureList.max() ?: 1.0)

        var lastPoint = points[0]
        for (i in 1..points.lastIndex) {
            val point = points[i]
            val curvature = curvatureList[i]
            g2d.color = Color.getHSBColor((curvature * curvatureScale).toFloat() , 1.0F, 1.0F)
            g2d.drawLine(lastPoint.x.toInt(), lastPoint.y.toInt(), point.x.toInt(), point.y.toInt())
            lastPoint = point
        }
    }
}