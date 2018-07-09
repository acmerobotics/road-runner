package com.acmerobotics.splinelib

import java.awt.GridLayout
import javax.swing.*


class PathPanel : JPanel() {
    init {
        layout = GridLayout(0, 4, 5, 0)

        add(JLabel("X", SwingConstants.CENTER))
        add(JLabel("Y", SwingConstants.CENTER))
        add(JLabel("Heading", SwingConstants.CENTER))
        add(JLabel("Reversed", SwingConstants.CENTER))

        add(JTextField(7))
        add(JTextField(7))
        add(JLabel())
        add(JCheckBox())
    }
}