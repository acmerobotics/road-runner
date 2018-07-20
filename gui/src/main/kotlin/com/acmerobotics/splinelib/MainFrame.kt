package com.acmerobotics.splinelib

import java.awt.Dimension
import javax.swing.JFrame
import javax.swing.JPanel

class MainFrame : JFrame() {
    init {
        title = "Spline Designer"
        size = Dimension(600, 1000)
        defaultCloseOperation = JFrame.EXIT_ON_CLOSE
        isResizable = false

        contentPane = JPanel()
        contentPane.add(MainPanel())
    }
}