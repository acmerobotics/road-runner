package com.acmerobotics.splinelib

import java.awt.Dimension
import javax.swing.BoxLayout
import javax.swing.JFrame
import javax.swing.JPanel

class MainFrame : JFrame() {
    init {
        title = "Spline Designer"
        size = Dimension(1000, 600)
        defaultCloseOperation = JFrame.EXIT_ON_CLOSE

        val panel = JPanel()
        val panel2 = JPanel()
        val layout = BoxLayout(panel, BoxLayout.LINE_AXIS)
        panel.layout = layout
        panel2.add(PathPanel())
        panel.add(panel2)
        panel.add(FieldPanel())
        contentPane = JPanel()
        contentPane.add(panel)
    }
}