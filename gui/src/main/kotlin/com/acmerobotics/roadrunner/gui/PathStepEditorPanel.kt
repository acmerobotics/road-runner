package com.acmerobotics.roadrunner.gui

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig
import java.awt.Color
import java.awt.Dimension
import java.awt.GridLayout
import javax.swing.*
import kotlin.math.min

private val INTERP_MAP = mapOf(
    "tangent" to TrajectoryConfig.HeadingInterpolationType.TANGENT,
    "constant" to TrajectoryConfig.HeadingInterpolationType.CONSTANT,
    "linear" to TrajectoryConfig.HeadingInterpolationType.LINEAR,
    "spline" to TrajectoryConfig.HeadingInterpolationType.SPLINE
)

private fun makeNumField(initialVal: Double): JTextField {
    val numField = JTextField(String.format("%.2f", initialVal))
    numField.horizontalAlignment = SwingConstants.CENTER
    return numField
}

/**
 * Panel for specifying the steps of the path/trajectory.
 */
class PathStepEditorPanel : JPanel() {
    override fun getPreferredSize(): Dimension {
        val size = super.getPreferredSize()

        return Dimension(size.width, min(150, size.height))
    }

    private class MutablePose2d(var x: Double, var y: Double, var heading: Double) {
        constructor(pose: Pose2d) : this(pose.x, pose.y, pose.heading)

        fun immutable() = Pose2d(x, y, heading)
    }

    private class MutableStep(
        var pose: MutablePose2d,
        var heading: Double?,
        var interpolationType: TrajectoryConfig.HeadingInterpolationType
    ) {
        constructor(step: TrajectoryConfig.Step) : this(MutablePose2d(step.pose), step.heading, step.interpolationType)

        fun immutable() = TrajectoryConfig.Step(pose.immutable(), heading, interpolationType)
    }

    private var startPose = MutablePose2d(0.0, 0.0, 0.0)
    private var startHeading: Double? = null
    private val steps = mutableListOf<MutableStep>()

    var onUpdateListener: ((Pose2d, Double?, List<TrajectoryConfig.Step>) -> Unit)? = null

    private val headerPanel = JPanel()
    private val scrollPanel = JPanel()
    private val startXField = makeNumField(startPose.x)
    private val startYField = makeNumField(startPose.y)
    private val startTangentField = makeNumField(startPose.heading.toDegrees())
    private val startHeadingField = makeNumField(startPose.heading.toDegrees())
    private val stepComponents = mutableListOf<List<JComponent>>()

    private var ignoreHeadingChange = false

    var trajectoryValid: Boolean = false
        set(value) {
            border = if (value) {
                null
            } else {
                BorderFactory.createLineBorder(Color.red)
            }
            field = value
        }

    init {
        headerPanel.layout = GridLayout(1, 7, 5, 0)
        scrollPanel.layout = GridLayout(0, 7, 5, 0)

        headerPanel.add(JLabel("Step", SwingConstants.CENTER))
        headerPanel.add(JLabel("X", SwingConstants.CENTER))
        headerPanel.add(JLabel("Y", SwingConstants.CENTER))
        headerPanel.add(JLabel("Tangent", SwingConstants.CENTER))
        headerPanel.add(JLabel("Interp", SwingConstants.CENTER))
        headerPanel.add(JLabel("Heading", SwingConstants.CENTER))
        val addButton = JButton("Add")
        addButton.addActionListener {
            addStep(steps.lastOrNull()?.immutable()
                ?: TrajectoryConfig.Step(startPose.immutable(), startHeading,
                    TrajectoryConfig.HeadingInterpolationType.TANGENT))

            fireUpdate()
        }
        headerPanel.add(addButton)

        scrollPanel.add(JLabel("1", SwingConstants.CENTER))
        scrollPanel.add(startXField)
        scrollPanel.add(startYField)
        scrollPanel.add(startTangentField)
        scrollPanel.add(JLabel())
        scrollPanel.add(startHeadingField)
        scrollPanel.add(JLabel())

        startXField.addChangeListener {
            startPose.x = startXField.text.toDoubleOrNull() ?: return@addChangeListener

            fireUpdate()
        }
        startYField.addChangeListener {
            startPose.y = startYField.text.toDoubleOrNull() ?: return@addChangeListener

            fireUpdate()
        }
        startTangentField.addChangeListener {
            startPose.heading = startTangentField.text.toDoubleOrNull()?.toRadians() ?: return@addChangeListener

            if (startHeading == null) {
                ignoreHeadingChange = true

                startHeadingField.text = String.format("%.2f", startPose.heading.toDegrees())

                ignoreHeadingChange = false

                revalidate()
            }

            fireUpdate()
        }
        startHeadingField.addChangeListener {
            if (ignoreHeadingChange) {
                return@addChangeListener
            }

            startHeading = startHeadingField.text.toDoubleOrNull()?.toRadians() ?: return@addChangeListener


            fireUpdate()
        }

        layout = BoxLayout(this, BoxLayout.PAGE_AXIS)

        val scrollPane = JScrollPane(ScrollPanelHost(scrollPanel))
        scrollPane.border = BorderFactory.createEmptyBorder()

        add(headerPanel)
        add(scrollPane)

        fireUpdate()
    }

    private fun fireUpdate() {
        onUpdateListener?.invoke(startPose.immutable(), startHeading, steps.map { it.immutable() })
    }

    private fun addStep(step: TrajectoryConfig.Step) {
        val mutableStep = MutableStep(step)

        val stepNumber = JLabel((steps.size + 2).toString())
        stepNumber.horizontalAlignment = SwingConstants.CENTER

        val xField = makeNumField(step.pose.x)
        val yField = makeNumField(step.pose.y)
        val tangentField = makeNumField(step.pose.heading.toDegrees())
        val interpComboBox = JComboBox(INTERP_MAP.keys.toTypedArray())
        interpComboBox.selectedItem = INTERP_MAP.entries.first { it.value == step.interpolationType }.key
        val headingField = makeNumField(step.heading?.toDegrees() ?: step.pose.heading.toDegrees())

        xField.addChangeListener {
            mutableStep.pose.x = xField.text.toDoubleOrNull() ?: return@addChangeListener

            fireUpdate()
        }
        yField.addChangeListener {
            mutableStep.pose.y = yField.text.toDoubleOrNull() ?: return@addChangeListener

            fireUpdate()
        }
        tangentField.addChangeListener {
            mutableStep.pose.heading = tangentField.text.toDoubleOrNull()?.toRadians() ?: return@addChangeListener

            fireUpdate()
        }

        interpComboBox.addActionListener {
            mutableStep.interpolationType = INTERP_MAP.getValue(interpComboBox.selectedItem as String)

            when (mutableStep.interpolationType) {
                TrajectoryConfig.HeadingInterpolationType.TANGENT -> {
                    headingField.isVisible = false
                }
                TrajectoryConfig.HeadingInterpolationType.CONSTANT -> {
                    headingField.isVisible = false
                }
                TrajectoryConfig.HeadingInterpolationType.LINEAR -> {
                    headingField.isVisible = true

                    mutableStep.heading = 0.0

                    headingField.text = "0.00"
                }
                TrajectoryConfig.HeadingInterpolationType.SPLINE -> {
                    headingField.isVisible = true

                    mutableStep.heading = 0.0

                    headingField.text = "0.00"
                }
            }

            fireUpdate()

            revalidate()
        }

        if (step.interpolationType == TrajectoryConfig.HeadingInterpolationType.TANGENT ||
            step.interpolationType == TrajectoryConfig.HeadingInterpolationType.CONSTANT) {
            headingField.isVisible = false
        }

        headingField.addChangeListener {
            mutableStep.heading = headingField.text.toDoubleOrNull()?.toRadians() ?: return@addChangeListener

            fireUpdate()
        }

        val removeButton = JButton("Remove")

        val uiComponents = listOf<JComponent>(
            stepNumber, xField, yField, tangentField, interpComboBox, headingField, removeButton)
        for (comp in uiComponents) {
            scrollPanel.add(comp)
        }

        steps.add(mutableStep)
        stepComponents.add(uiComponents)

        removeButton.addActionListener {
            removeStep(mutableStep)

            fireUpdate()
        }

        revalidate()
    }

    private fun removeStep(step: MutableStep) {
        removeStepAt(steps.indexOf(step))
    }

    private fun removeStepAt(index: Int) {
        for (comp in stepComponents[index]) {
            scrollPanel.remove(comp)
        }
        steps.removeAt(index)
        stepComponents.removeAt(index)

        for (i in index until stepComponents.size) {
            val step = stepComponents[i][0] as JLabel
            step.text = (i + 2).toString()
        }

        revalidate()

        fireUpdate()
    }

    fun update(newStartPose: Pose2d, newStartHeading: Double?, newSteps: List<TrajectoryConfig.Step>) {
        steps.clear()

        for (component in stepComponents.flatten()) {
            scrollPanel.remove(component)
        }
        stepComponents.clear()

        for (step in newSteps) {
            addStep(step)
        }

        startPose = MutablePose2d(newStartPose)
        startHeading = newStartHeading

        startXField.text = String.format("%.2f", startPose.x)
        startYField.text = String.format("%.2f", startPose.y)
        startTangentField.text = String.format("%.2f", startPose.heading.toDegrees())
        if (startHeading != null) {
            startHeadingField.text = String.format("%.2f", startHeading!!.toDegrees())
        }

        fireUpdate()
    }
}
