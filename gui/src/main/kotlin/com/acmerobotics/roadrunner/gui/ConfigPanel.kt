package com.acmerobotics.roadrunner.gui

import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import java.awt.GridLayout
import javax.swing.*

private val DRIVE_MAP = mapOf(
    "generic" to TrajectoryGroupConfig.DriveType.GENERIC,
    "mecanum" to TrajectoryGroupConfig.DriveType.MECANUM,
    "tank" to TrajectoryGroupConfig.DriveType.TANK
)

private val UNIT_MAP = mapOf(
    "foot" to TrajectoryGroupConfig.DistanceUnit.FOOT,
    "inch" to TrajectoryGroupConfig.DistanceUnit.INCH,
    "meter" to TrajectoryGroupConfig.DistanceUnit.METER,
    "centimeter" to TrajectoryGroupConfig.DistanceUnit.CENTIMETER,
    "millimeter" to TrajectoryGroupConfig.DistanceUnit.MILLIMETER
)

// TODO: make some kind of wrapper around JTextField for numeric fields
private fun makeNumField(initialVal: Double) = JTextField(String.format("%.2f", initialVal))

/**
 * Panel for specifying the robot kinematic constraints.
 */
class ConfigPanel : JPanel() {
    private class MutableDriveConstraints(
        var maximumVelocity: Double,
        var maximumAcceleration: Double,
        var maximumAngularVelocity: Double,
        var maximumAngularAcceleration: Double
    ) {
        constructor(constraints: DriveConstraints) : this(
            constraints.maxVel,
            constraints.maxAccel,
            constraints.maxAngVel,
            constraints.maxAngAccel
        )

        fun immutable(): DriveConstraints = DriveConstraints(
            maximumVelocity,
            maximumAcceleration,
            0.0,
            maximumAngularVelocity,
            maximumAngularAcceleration,
            0.0
        )
    }

    private var distanceUnit = TrajectoryGroupConfig.DistanceUnit.INCH
    private var driveType = TrajectoryGroupConfig.DriveType.GENERIC
    private var trackWidth: Double? = null
    private var wheelBase: Double? = null
    private var lateralMultiplier: Double? = null

    private var mutableConstraints: MutableDriveConstraints = MutableDriveConstraints(0.0, 0.0, 0.0, 0.0)

    var onUpdateListener: ((TrajectoryGroupConfig) -> Unit)? = null

    private val distanceUnitComboBox = JComboBox(UNIT_MAP.keys.toTypedArray())
    private val driveTypeComboBox = JComboBox(DRIVE_MAP.keys.toTypedArray())
    private val trackWidthLabel = JLabel("Track Width", SwingConstants.RIGHT)
    private val trackWidthField = makeNumField(0.0)
    private val wheelBaseLabel = JLabel("Wheel Base", SwingConstants.RIGHT)
    private val wheelBaseField = makeNumField(0.0)
    private val lateralMultiplierLabel = JLabel("Lateral Multiplier", SwingConstants.RIGHT)
    private val lateralMultiplierField = makeNumField(0.0)

    private val maxVelTextField = makeNumField(0.0)
    private val maxAccelTextField = makeNumField(0.0)
    private val maxAngVelTextField = makeNumField(0.0)
    private val maxAngAccelTextField = makeNumField(0.0)

    private var ignoreWheelBaseChanges = false

    // TODO: make some helpers to make Swing less awful
    init {
        layout = BoxLayout(this, BoxLayout.LINE_AXIS)

        val leftPanel = JPanel()
        leftPanel.layout = GridLayout(0, 2, 5, 5)

        leftPanel.add(JLabel("Distance Unit", SwingConstants.RIGHT))
        leftPanel.add(distanceUnitComboBox)
        distanceUnitComboBox.addActionListener {
            distanceUnit = UNIT_MAP.getValue(distanceUnitComboBox.selectedItem as String)

            fireUpdate()
        }

        leftPanel.add(JLabel("Drive Type", SwingConstants.RIGHT))
        leftPanel.add(driveTypeComboBox)
        driveTypeComboBox.addActionListener {
            driveType = DRIVE_MAP.getValue(driveTypeComboBox.selectedItem as String)
            when (driveType) {
                TrajectoryGroupConfig.DriveType.GENERIC -> {
                    trackWidthLabel.isVisible = false
                    trackWidthField.isVisible = false
                    wheelBaseLabel.isVisible = false
                    wheelBaseField.isVisible = false
                    lateralMultiplierLabel.isVisible = false
                    lateralMultiplierField.isVisible = false

                    trackWidth = null
                    wheelBase = null
                    lateralMultiplier = null
                }
                TrajectoryGroupConfig.DriveType.MECANUM -> {
                    trackWidthLabel.isVisible = true
                    trackWidthField.isVisible = true
                    wheelBaseLabel.isVisible = true
                    wheelBaseField.isVisible = true
                    lateralMultiplierLabel.isVisible = true
                    lateralMultiplierField.isVisible = true

                    trackWidth = 18.0
                    wheelBase = null
                    lateralMultiplier = 1.0

                    trackWidthField.text = "18.00"

                    ignoreWheelBaseChanges = true
                    wheelBaseField.text = "18.00"
                    ignoreWheelBaseChanges = false

                    lateralMultiplierField.text = "1.00"
                }
                TrajectoryGroupConfig.DriveType.TANK -> {
                    trackWidthLabel.isVisible = true
                    trackWidthField.isVisible = true
                    wheelBaseLabel.isVisible = false
                    wheelBaseField.isVisible = false
                    lateralMultiplierLabel.isVisible = false
                    lateralMultiplierField.isVisible = false

                    trackWidth = 18.0
                    wheelBase = null
                    lateralMultiplier = null

                    trackWidthField.text = "18.00"
                }
            }

            fireUpdate()
        }

        leftPanel.add(trackWidthLabel)
        leftPanel.add(trackWidthField)
        trackWidthField.addChangeListener {
            trackWidth = trackWidthField.text.toDoubleOrNull() ?: return@addChangeListener

            if (wheelBase == null) {
                ignoreWheelBaseChanges = true

                wheelBaseField.text = String.format("%.2f", trackWidth)

                ignoreWheelBaseChanges = false

                revalidate()
            }

            fireUpdate()
        }

        leftPanel.add(wheelBaseLabel)
        leftPanel.add(wheelBaseField)
        wheelBaseField.addChangeListener {
            if (ignoreWheelBaseChanges) {
                return@addChangeListener
            }

            wheelBase = wheelBaseField.text.toDoubleOrNull() ?: return@addChangeListener

            fireUpdate()
        }

        leftPanel.add(lateralMultiplierLabel)
        leftPanel.add(lateralMultiplierField)
        lateralMultiplierField.addChangeListener {
            lateralMultiplier = lateralMultiplierField.text.toDoubleOrNull() ?: return@addChangeListener

            fireUpdate()
        }

        val rightPanel = JPanel()
        rightPanel.layout = GridLayout(0, 2, 5, 5)

        rightPanel.add(JLabel("Max Velocity", SwingConstants.RIGHT))
        maxVelTextField.addChangeListener {
            mutableConstraints.maximumVelocity = maxVelTextField.text.toDoubleOrNull()
                ?: return@addChangeListener

            fireUpdate()
        }
        rightPanel.add(maxVelTextField)

        rightPanel.add(JLabel("Max Accel", SwingConstants.RIGHT))
        maxAccelTextField.addChangeListener {
            mutableConstraints.maximumAcceleration = maxAccelTextField.text.toDoubleOrNull()
                ?: return@addChangeListener

            fireUpdate()
        }
        rightPanel.add(maxAccelTextField)

        rightPanel.add(JLabel("Max Ang Velocity", SwingConstants.RIGHT))
        maxAngVelTextField.addChangeListener {
            mutableConstraints.maximumAngularVelocity = maxAngVelTextField.text.toDoubleOrNull()?.toRadians()
                ?: return@addChangeListener

            fireUpdate()
        }
        rightPanel.add(maxAngVelTextField)

        rightPanel.add(JLabel("Max Ang Accel", SwingConstants.RIGHT))
        maxAngAccelTextField.addChangeListener {
            mutableConstraints.maximumAngularAcceleration = maxAngAccelTextField.text.toDoubleOrNull()?.toRadians()
                ?: return@addChangeListener

            fireUpdate()
        }
        rightPanel.add(maxAngAccelTextField)

        add(Box.createHorizontalGlue())
        add(leftPanel)
        add(Box.createHorizontalGlue())
        add(rightPanel)
        add(Box.createHorizontalGlue())
    }

    private fun fireUpdate() {
        onUpdateListener?.invoke(TrajectoryGroupConfig(mutableConstraints.immutable(), distanceUnit, driveType, trackWidth, wheelBase, lateralMultiplier))
    }

    fun update(groupConfig: TrajectoryGroupConfig) {
        val constraints = groupConfig.constraints
        this.mutableConstraints = MutableDriveConstraints(constraints)

        driveTypeComboBox.selectedItem = DRIVE_MAP.entries.first { it.value == driveType }.key

        distanceUnit = groupConfig.distanceUnit
        driveType = groupConfig.driveType
        trackWidth = groupConfig.trackWidth
        wheelBase = groupConfig.wheelBase
        lateralMultiplier = groupConfig.lateralMultiplier

        distanceUnitComboBox.selectedItem = UNIT_MAP.entries.first { it.value == distanceUnit }.key
        trackWidthField.text = String.format("%.2f", trackWidth)
        if (wheelBase != null) {
            wheelBaseField.text = String.format("%.2f", wheelBase!!)
        }
        if (lateralMultiplier != null) {
            lateralMultiplierField.text = String.format("%.2f", lateralMultiplier)
        }

        maxVelTextField.text = String.format("%.2f", constraints.maxVel)
        maxAccelTextField.text = String.format("%.2f", constraints.maxAccel)
        maxAngVelTextField.text = String.format("%.2f", constraints.maxAngVel.toDegrees())
        maxAngAccelTextField.text = String.format("%.2f", constraints.maxAngAccel.toDegrees())
    }
}
