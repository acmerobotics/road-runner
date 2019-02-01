package com.acmerobotics.roadrunner.gui

import com.acmerobotics.roadrunner.trajectory.Trajectory
import java.awt.GridLayout
import javax.swing.JLabel
import javax.swing.JPanel
import javax.swing.JTextField
import javax.swing.SwingConstants

class TrajectoryInfoPanel : JPanel() {
    private val durationLabel: JLabel
    private val resolutionTextField: JTextField
    private var resolution = 0.0

    var onResolutionUpdateListener: ((Double) -> Unit)? = null

    init {
        val panel = JPanel()
        panel.layout = GridLayout(0, 3, 5, 5)

        durationLabel = JLabel("Duration: 0.00s")
        panel.add(durationLabel)

        panel.add(JLabel("Resolution:", SwingConstants.RIGHT))
        resolutionTextField = JTextField(resolution.toString(), SwingConstants.LEFT)
        resolutionTextField.addChangeListener {
            resolution = resolutionTextField.text.toDoubleOrNull() ?: resolution
            onResolutionUpdateListener?.invoke(resolution)
        }
        panel.add(resolutionTextField)

        add(panel)
    }

    fun updateTrajectory(trajectory: Trajectory) {
        durationLabel.text = if (trajectory.duration().isInfinite() || trajectory.duration().isNaN()) {
            "Duration: --"
        } else {
            String.format("Duration: %.2fs", trajectory.duration())
        }
    }

    fun updateResolution(resolution: Double) {
        resolutionTextField.text = resolution.toString()
    }
}