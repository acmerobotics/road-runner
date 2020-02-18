package com.acmerobotics.roadrunner.gui

import DEFAULT_GROUP_CONFIG
import DEFAULT_TRACK_WIDTH
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig
import java.awt.Dimension
import java.awt.GridLayout
import javax.swing.*
/**
 * Panel for specifying the robot kinematic constraints.
 */
class ConfigPanel : JPanel() {
    private val maxVelTextField = makeFormattedDoubleField()
    private val maxAccelTextField = makeFormattedDoubleField()
    private val maxAngVelTextField = makeFormattedDoubleField()
    private val maxAngAccelTextField = makeFormattedDoubleField()
    private val robotLengthField = makeFormattedDoubleField()
    private val robotWidthField = makeFormattedDoubleField()
    private val driveTypeComboBox = JComboBox(TrajectoryGroupConfig.DriveType.values())
    private val trackWidthLabel = JLabel("Track Width", SwingConstants.RIGHT)
    private val trackWidthField = makeFormattedDoubleField()
    private val wheelBaseLabel = JLabel("Wheel Base", SwingConstants.RIGHT)
    private val wheelBaseField = makeFormattedDoubleField()
    private val lateralMultiplierLabel = JLabel("Lateral Multiplier", SwingConstants.RIGHT)
    private val lateralMultiplierField = makeFormattedDoubleField()

    var onConfigUpdate: ((TrajectoryGroupConfig) -> Unit)? = null
    private var externalUpdate = false
    private var updating = false
    private var _config: TrajectoryGroupConfig = DEFAULT_GROUP_CONFIG
        set(value) {
            if (updating) {
                return
            }

            updating = true

            if (maxVelTextField.doubleValue != value.maxVel) {
                maxVelTextField.value = value.maxVel
            }
            if (maxAccelTextField.doubleValue != value.maxAccel) {
                maxAccelTextField.value = value.maxAccel
            }
            if (maxAngVelTextField.doubleValue != value.maxAngVel.toDegrees()) {
                maxAngVelTextField.value = value.maxAngVel.toDegrees()
            }
            if (maxAngAccelTextField.doubleValue != value.maxAngAccel.toDegrees()) {
                maxAngAccelTextField.value = value.maxAngAccel.toDegrees()
            }

            if (robotLengthField.doubleValue != value.robotLength) {
                robotLengthField.value = value.robotLength
            }
            if (robotWidthField.doubleValue != value.robotWidth) {
                robotWidthField.value = value.robotWidth
            }

            if (driveTypeComboBox.selectedItemTyped != value.driveType) {
                driveTypeComboBox.selectedItem = value.driveType
            }
            when (value.driveType) {
                TrajectoryGroupConfig.DriveType.GENERIC -> {
                    trackWidthLabel.isVisible = false
                    trackWidthField.isVisible = false
                    wheelBaseLabel.isVisible = false
                    wheelBaseField.isVisible = false
                    lateralMultiplierLabel.isVisible = false
                    lateralMultiplierField.isVisible = false
                }
                TrajectoryGroupConfig.DriveType.MECANUM -> {
                    trackWidthLabel.isVisible = true
                    trackWidthField.isVisible = true
                    wheelBaseLabel.isVisible = true
                    wheelBaseField.isVisible = true
                    lateralMultiplierLabel.isVisible = true
                    lateralMultiplierField.isVisible = true
                }
                TrajectoryGroupConfig.DriveType.TANK -> {
                    trackWidthLabel.isVisible = true
                    trackWidthField.isVisible = true
                    wheelBaseLabel.isVisible = false
                    wheelBaseField.isVisible = false
                    lateralMultiplierLabel.isVisible = false
                    lateralMultiplierField.isVisible = false
                }
            }

            if (trackWidthField.doubleValue != value.trackWidth) {
                trackWidthField.value = value.trackWidth
            }
            if (value.wheelBase == null) {
                if (wheelBaseField.doubleValue != value.trackWidth) {
                    wheelBaseField.value = value.trackWidth
                }
            } else {
                if (wheelBaseField.doubleValue != value.wheelBase) {
                    wheelBaseField.value = value.wheelBase
                }
            }
            if (value.lateralMultiplier != null) {
                if (lateralMultiplierField.doubleValue != value.lateralMultiplier) {
                    lateralMultiplierField.value = value.lateralMultiplier
                }
            }

            field = value

            if (!externalUpdate) {
                onConfigUpdate?.invoke(value)
            }

            updating = false
        }
    var config
        get() = _config
        set(value) {
            externalUpdate = true

            _config = value

            externalUpdate = false
        }

    init {
        layout = BoxLayout(this, BoxLayout.LINE_AXIS)

        val leftPanel = JPanel()
        leftPanel.layout = GridLayout(0, 2, 5, 5)

        leftPanel.add(JLabel("Robot Length", SwingConstants.RIGHT))
        leftPanel.add(robotLengthField)
        robotLengthField.addValueChangeListener<Number> {
            _config = _config.copy(robotLength = it.toDouble())
        }

        leftPanel.add(JLabel("Robot Width", SwingConstants.RIGHT))
        leftPanel.add(robotWidthField)
        robotWidthField.addValueChangeListener<Number> {
            _config = _config.copy(robotWidth = it.toDouble())
        }

        leftPanel.add(JLabel("Drive Type", SwingConstants.RIGHT))
        leftPanel.add(driveTypeComboBox)
        driveTypeComboBox.addActionListener {
            val newDriveType = driveTypeComboBox.selectedItemTyped ?: return@addActionListener
            if (_config.driveType != newDriveType) {
                _config = when (newDriveType) {
                    TrajectoryGroupConfig.DriveType.GENERIC -> {
                        _config.copy(
                            driveType = TrajectoryGroupConfig.DriveType.GENERIC,
                            trackWidth = null, wheelBase = null, lateralMultiplier = null
                        )
                    }
                    TrajectoryGroupConfig.DriveType.TANK -> {
                        _config.copy(
                            driveType = TrajectoryGroupConfig.DriveType.TANK,
                            trackWidth = DEFAULT_TRACK_WIDTH, wheelBase = null, lateralMultiplier = null
                        )
                    }
                    TrajectoryGroupConfig.DriveType.MECANUM -> {
                        _config.copy(
                            driveType = TrajectoryGroupConfig.DriveType.MECANUM,
                            trackWidth = DEFAULT_TRACK_WIDTH, wheelBase = null, lateralMultiplier = 1.0
                        )
                    }
                }
            }
        }

        leftPanel.add(trackWidthLabel)
        leftPanel.add(trackWidthField)
        trackWidthField.addValueChangeListener<Number> {
            _config = _config.copy(trackWidth = it.toDouble())
        }

        leftPanel.add(wheelBaseLabel)
        leftPanel.add(wheelBaseField)
        wheelBaseField.addValueChangeListener<Number> {
            _config = _config.copy(wheelBase = it.toDouble())
        }

        leftPanel.add(lateralMultiplierLabel)
        leftPanel.add(lateralMultiplierField)
        lateralMultiplierField.addValueChangeListener<Number> {
            _config = _config.copy(lateralMultiplier = it.toDouble())
        }

        leftPanel.maximumSize = leftPanel.preferredSize
        leftPanel.minimumSize = Dimension(0, leftPanel.minimumSize.height)

        val rightPanel = JPanel()
        rightPanel.layout = GridLayout(0, 2, 5, 5)

        rightPanel.add(JLabel("Max Velocity", SwingConstants.RIGHT))
        maxVelTextField.addValueChangeListener<Number> {
            _config = _config.copy(maxVel = it.toDouble())
        }
        rightPanel.add(maxVelTextField)

        rightPanel.add(JLabel("Max Accel", SwingConstants.RIGHT))
        maxAccelTextField.addValueChangeListener<Number> {
            _config = _config.copy(maxAccel = it.toDouble())
        }
        rightPanel.add(maxAccelTextField)

        rightPanel.add(JLabel("Max Ang Velocity", SwingConstants.RIGHT))
        maxAngVelTextField.addValueChangeListener<Number> {
            _config = _config.copy(maxAngVel = it.toDouble().toRadians())
        }
        rightPanel.add(maxAngVelTextField)

        rightPanel.add(JLabel("Max Ang Accel", SwingConstants.RIGHT))
        maxAngAccelTextField.addValueChangeListener<Number> {
            _config = _config.copy(maxAngAccel = it.toDouble().toRadians())
        }
        rightPanel.add(maxAngAccelTextField)

        rightPanel.maximumSize = rightPanel.preferredSize
        rightPanel.minimumSize = Dimension(0, rightPanel.minimumSize.height)

        add(Box.createHorizontalGlue())
        add(leftPanel)
        add(Box.createHorizontalGlue())
        add(rightPanel)
        add(Box.createHorizontalGlue())

        config = DEFAULT_GROUP_CONFIG
    }
}
