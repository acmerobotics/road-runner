package com.acmerobotics.roadrunner.gui

import DEFAULT_ROBOT_SIZE
import com.acmerobotics.roadrunner.trajectory.Trajectory
import javax.swing.JFrame

class TrajectoryFrame(trajectory: Trajectory): JFrame() {
    init {
        title = String.format("Road Runner GUI - %.2fs", trajectory.duration())
        defaultCloseOperation = EXIT_ON_CLOSE
        setLocationRelativeTo(null)

        val panel = FieldPanel()
        panel.robotDimensions = RobotDimensions(DEFAULT_ROBOT_SIZE, DEFAULT_ROBOT_SIZE)
        panel.trajectory = trajectory
        add(panel)

        pack()
    }
}
