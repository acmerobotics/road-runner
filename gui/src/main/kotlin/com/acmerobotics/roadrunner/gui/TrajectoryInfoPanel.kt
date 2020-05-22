package com.acmerobotics.roadrunner.gui

import DEFAULT_RESOLUTION
import java.awt.GridLayout
import javax.swing.JLabel
import javax.swing.JPanel
import javax.swing.SwingConstants

/**
 * Panel with basic information about the generated trajectory and some configurable options.
 */
class TrajectoryInfoPanel : JPanel() {
    private val durationLabel = JLabel()
    private val resolutionTextField = makeFormattedDoubleField()

    var duration: Double = 0.0
        set(value) {
            durationLabel.text = if (value.isInfinite() || value.isNaN()) {
                "Duration: --"
            } else {
                String.format("Duration: %.2fs", value)
            }
            field = value
        }

    var onResolutionChange: ((Double) -> Unit)? = null
    private var externalUpdate = false
    private var updating = false
    private var _resolution: Double = 0.0
        set(value) {
            if (updating) {
                return
            }

            updating = true

            if (resolutionTextField.doubleValue != value) {
                resolutionTextField.value = value
            }

            field = value

            if (!externalUpdate) {
                onResolutionChange?.invoke(value)
            }

            updating = false
        }
    var resolution
        get() = _resolution
        set(value) {
            externalUpdate = true

            _resolution = value

            externalUpdate = false
        }

    init {
        val panel = JPanel()
        panel.layout = GridLayout(0, 3, 5, 5)

        duration = 0.0
        panel.add(durationLabel)

        panel.add(JLabel("Resolution:", SwingConstants.RIGHT))
        resolutionTextField.horizontalAlignment = SwingConstants.LEFT
        resolutionTextField.addValueChangeListener<Number> {
            _resolution = it.toDouble()
        }
        panel.add(resolutionTextField)

        add(panel)

        maximumSize = preferredSize

        resolution = DEFAULT_RESOLUTION
    }
}
