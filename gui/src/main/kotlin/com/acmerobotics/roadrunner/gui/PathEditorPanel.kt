package com.acmerobotics.roadrunner.gui

import DEFAULT_STEP
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig
import java.awt.BorderLayout
import java.awt.Color
import java.awt.Dimension
import java.awt.GridLayout
import javax.swing.*
import kotlin.math.min


class WidthAgnosticPanel : JPanel() {
    override fun getPreferredSize(): Dimension {
        val baseSize = super.getPreferredSize()
        return Dimension(0, baseSize.height)
    }
}

class PathStepPanel : JPanel() {
    private val indexLabel = JLabel("", SwingConstants.CENTER)
    private val xTextField = makeFormattedDoubleField()
    private val yTextField = makeFormattedDoubleField()
    private val tangentTextField = makeFormattedDoubleField()
    private val interpComboBox = JComboBox(TrajectoryConfig.HeadingInterpolationType.values())
    private val headingTextField = makeFormattedDoubleField()
    private val removeButton = JButton("Remove")

    var index: Int? = null
        set(value) {
            indexLabel.text = (value?.plus(2)).toString()
            field = value
        }

    var onStepRemove: (() -> Unit)? = null

    var onStepUpdate: ((TrajectoryConfig.Step) -> Unit)? = null
    private var externalUpdate = false
    private var updating = false
    private var _step: TrajectoryConfig.Step = DEFAULT_STEP
        set(value) {
            if (updating) {
                return
            }

            updating = true

            if (xTextField.doubleValue != value.pose.x) {
                xTextField.value = value.pose.x
            }
            if (yTextField.doubleValue != value.pose.y) {
                yTextField.value = value.pose.y
            }
            if (tangentTextField.doubleValue != value.pose.heading.toDegrees()) {
                tangentTextField.value = value.pose.heading.toDegrees()
            }

            val newHeading = value.heading
            if (newHeading != null) {
                if (headingTextField.doubleValue != newHeading.toDegrees()) {
                    headingTextField.value = newHeading.toDegrees()
                }
            }

            if (interpComboBox.selectedItemTyped != value.interpolationType) {
                interpComboBox.selectedItem = value.interpolationType
            }
            headingTextField.isVisible = value.interpolationType == TrajectoryConfig.HeadingInterpolationType.LINEAR
                || value.interpolationType == TrajectoryConfig.HeadingInterpolationType.SPLINE

            field = value

            if (!externalUpdate) {
                onStepUpdate?.invoke(value)
            }

            updating = false
        }
    var step
        get() = _step
        set(value) {
            externalUpdate = true

            _step = value

            externalUpdate = false
        }

    init {
        xTextField.addValueChangeListener<Number> {
            _step = _step.copy(pose = _step.pose.copy(x = it.toDouble()))
        }
        yTextField.addValueChangeListener<Number> {
            _step = _step.copy(pose = _step.pose.copy(y = it.toDouble()))
        }
        tangentTextField.addValueChangeListener<Number> {
            _step = _step.copy(pose = _step.pose.copy(heading = it.toDouble().toRadians()))
        }

        interpComboBox.addActionListener {
            val newInterpType = interpComboBox.selectedItemTyped ?: return@addActionListener
            if (_step.interpolationType != newInterpType) {
                _step = when (newInterpType) {
                    TrajectoryConfig.HeadingInterpolationType.TANGENT -> {
                        _step.copy(interpolationType = TrajectoryConfig.HeadingInterpolationType.TANGENT, heading = null)
                    }
                    TrajectoryConfig.HeadingInterpolationType.CONSTANT -> {
                        _step.copy(interpolationType = TrajectoryConfig.HeadingInterpolationType.CONSTANT, heading = null)
                    }
                    TrajectoryConfig.HeadingInterpolationType.LINEAR -> {
                        _step.copy(interpolationType = TrajectoryConfig.HeadingInterpolationType.LINEAR, heading = _step.pose.heading)
                    }
                    TrajectoryConfig.HeadingInterpolationType.SPLINE -> {
                        _step.copy(interpolationType = TrajectoryConfig.HeadingInterpolationType.SPLINE, heading = _step.pose.heading)
                    }
                }
            }
        }

        headingTextField.addValueChangeListener<Number> {
            _step = _step.copy(heading = it.toDouble().toRadians())
        }

        removeButton.addActionListener {
            onStepRemove?.invoke()
        }

        layout = GridLayout(1, 7, 5, 0)
        add(indexLabel)
        add(xTextField)
        add(yTextField)
        add(tangentTextField)
        add(interpComboBox)
        add(headingTextField)
        add(removeButton)

        maximumSize = Dimension(maximumSize.width, preferredSize.height)

        step = DEFAULT_STEP
    }
}

data class PathConfig(
    val startPose: Pose2d,
    val startHeading: Double?,
    val steps: List<TrajectoryConfig.Step>
)

private val DEFAULT_CONFIG = PathConfig(
    Pose2d(),
    null,
    emptyList()
)

class PathEditorPanel : JPanel() {
    private val xTextField = makeFormattedDoubleField()
    private val yTextField = makeFormattedDoubleField()
    private val tangentTextField = makeFormattedDoubleField()
    private val headingTextField = makeFormattedDoubleField()
    private val stepPanelContainer = WidthAgnosticPanel()
    private val stepPanels = mutableListOf<PathStepPanel>()
    private val headerContainer = JPanel()

    var onConfigUpdate: ((PathConfig) -> Unit)? = null
    private var externalUpdate = false
    private var updating = false
    private var _config: PathConfig = DEFAULT_CONFIG
        set(value) {
            if (updating) {
                return
            }

            updating = true

            if (xTextField.doubleValue != value.startPose.x) {
                xTextField.value = value.startPose.x
            }
            if (yTextField.doubleValue != value.startPose.y) {
                yTextField.value = value.startPose.y
            }
            if (tangentTextField.doubleValue != value.startPose.heading.toDegrees()) {
                tangentTextField.value = value.startPose.heading.toDegrees()
            }

            val newStartHeading = value.startHeading
            if (newStartHeading != null) {
                if (headingTextField.doubleValue != newStartHeading.toDegrees()) {
                    headingTextField.value = newStartHeading.toDegrees()
                }
            } else {
                if (headingTextField.doubleValue != value.startPose.heading.toDegrees()) {
                    headingTextField.value = value.startPose.heading.toDegrees()
                }
            }

            // update existing step panels
            for ((i, pair) in value.steps.zip(stepPanels).withIndex()) {
                val (step, panel) = pair
                panel.isVisible = true
                panel.onStepRemove = {
                    _config = _config.copy(steps = _config.steps.filterIndexed { j, _ -> i != j })
                }
                panel.onStepUpdate = { newStep ->
                    _config = _config.copy(steps = _config.steps.mapIndexed { j, step ->
                        if (i == j) {
                            newStep
                        } else {
                            step
                        }
                    })
                }
                if (panel.step !== step) {
                    panel.step = step
                }
            }

            // create additional panels if necessary
            val startSize = stepPanels.size
            for ((i, step) in value.steps.drop(startSize).withIndex()) {
                val panel = PathStepPanel()
                panel.index = i + startSize
                panel.step = step
                panel.onStepRemove = {
                    _config = _config.copy(steps = _config.steps.filterIndexed { j, _ -> i + startSize != j })
                }
                panel.onStepUpdate = { newStep ->
                    _config = _config.copy(steps = _config.steps.mapIndexed { j, step ->
                        if (i + startSize == j) {
                            newStep
                        } else {
                            step
                        }
                    })
                }
                stepPanelContainer.add(panel)
                stepPanels.add(panel)
            }

            // hide the rest
            for (stepPanel in stepPanels.drop(value.steps.size)) {
                stepPanel.isVisible = false
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

    var valid: Boolean = true
        set(value) {
            border = if (value) {
                BorderFactory.createEmptyBorder()
            } else {
                BorderFactory.createLineBorder(Color.RED, 2)
            }
            field = value
        }

    init {
        xTextField.addValueChangeListener<Number> {
            _config = _config.copy(startPose = _config.startPose.copy(x = it.toDouble()))
        }
        yTextField.addValueChangeListener<Number> {
            _config = _config.copy(startPose = _config.startPose.copy(y = it.toDouble()))
        }
        tangentTextField.addValueChangeListener<Number> {
            _config = _config.copy(startPose = _config.startPose.copy(heading = it.toDouble().toRadians()))
        }
        headingTextField.addValueChangeListener<Number> {
            _config = _config.copy(startHeading = it.toDouble().toRadians())
        }

        layout = BorderLayout()

        headerContainer.layout = GridLayout(1, 7, 5, 0)
        headerContainer.add(JLabel("Step", SwingConstants.CENTER))
        headerContainer.add(JLabel("X", SwingConstants.CENTER))
        headerContainer.add(JLabel("Y", SwingConstants.CENTER))
        headerContainer.add(JLabel("Tangent", SwingConstants.CENTER))
        headerContainer.add(JLabel("Interp", SwingConstants.CENTER))
        headerContainer.add(JLabel("Heading", SwingConstants.CENTER))

        val addButton = JButton("Add")
        addButton.addActionListener {
            _config = _config.copy(steps = _config.steps + listOf(_config.steps.lastOrNull()?.copy()
                ?: TrajectoryConfig.Step(_config.startPose.copy(), _config.startHeading, TrajectoryConfig.HeadingInterpolationType.TANGENT)
            ))
        }
        headerContainer.add(addButton)

        headerContainer.border = BorderFactory.createEmptyBorder(0, 0, 0, 15)
        add(headerContainer, BorderLayout.NORTH)

        val scrollPane = JScrollPane(stepPanelContainer, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS, JScrollPane.HORIZONTAL_SCROLLBAR_NEVER)
        scrollPane.border = BorderFactory.createEmptyBorder()

        stepPanelContainer.layout = BoxLayout(stepPanelContainer, BoxLayout.PAGE_AXIS)

        val firstRow = JPanel()
        firstRow.layout = GridLayout(1, 7, 5, 0)
        firstRow.add(JLabel("1", SwingConstants.CENTER))
        firstRow.add(xTextField)
        firstRow.add(yTextField)
        firstRow.add(tangentTextField)
        firstRow.add(JLabel())
        firstRow.add(headingTextField)
        firstRow.add(JLabel())

        firstRow.maximumSize = Dimension(firstRow.maximumSize.width, firstRow.preferredSize.height)

        stepPanelContainer.add(firstRow)

        add(scrollPane, BorderLayout.CENTER)

        config = DEFAULT_CONFIG
    }

    override fun getPreferredSize(): Dimension {
        val baseSize = super.getPreferredSize()
        return Dimension(baseSize.width, min(baseSize.height, 300))
    }

    override fun getMaximumSize(): Dimension {
        return Dimension(super.getMaximumSize().width, 300)
    }
}
