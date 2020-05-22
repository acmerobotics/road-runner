package com.acmerobotics.roadrunner.gui

import DEFAULT_WAYPOINT
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

class WaypointPanel : JPanel() {
    private val indexLabel = JLabel("", SwingConstants.CENTER)
    private val xTextField = makeFormattedDoubleField()
    private val yTextField = makeFormattedDoubleField()
    private val headingTextField = makeFormattedDoubleField()
    private val interpComboBox = JComboBox(TrajectoryConfig.HeadingInterpolationType.values())
    private val tangentTextField = makeFormattedDoubleField()
    private val removeButton = JButton("Remove")

    var index: Int? = null
        set(value) {
            indexLabel.text = (value?.plus(2)).toString()
            field = value
        }

    var onWaypointRemove: (() -> Unit)? = null

    var onWaypointChange: ((TrajectoryConfig.Waypoint) -> Unit)? = null
    private var externalUpdate = false
    private var updating = false
    private var _waypoint: TrajectoryConfig.Waypoint = DEFAULT_WAYPOINT
        set(value) {
            if (updating) {
                return
            }

            updating = true

            if (xTextField.doubleValue != value.position.x) {
                xTextField.value = value.position.x
            }
            if (yTextField.doubleValue != value.position.y) {
                yTextField.value = value.position.y
            }
            if (headingTextField.doubleValue != value.heading.toDegrees()) {
                headingTextField.value = value.heading.toDegrees()
            }
            if (tangentTextField.doubleValue != value.tangent.toDegrees()) {
                tangentTextField.value = value.tangent.toDegrees()
            }

            if (interpComboBox.selectedItemTyped != value.interpolationType) {
                interpComboBox.selectedItem = value.interpolationType
            }
            headingTextField.isVisible = value.interpolationType == TrajectoryConfig.HeadingInterpolationType.LINEAR ||
                value.interpolationType == TrajectoryConfig.HeadingInterpolationType.SPLINE

            field = value

            if (!externalUpdate) {
                onWaypointChange?.invoke(value)
            }

            updating = false
        }
    var waypoint
        get() = _waypoint
        set(value) {
            externalUpdate = true

            _waypoint = value

            externalUpdate = false
        }

    init {
        xTextField.addValueChangeListener<Number> {
            _waypoint = _waypoint.copy(position = _waypoint.position.copy(x = it.toDouble()))
        }
        yTextField.addValueChangeListener<Number> {
            _waypoint = _waypoint.copy(position = _waypoint.position.copy(y = it.toDouble()))
        }
        headingTextField.addValueChangeListener<Number> {
            _waypoint = _waypoint.copy(heading = it.toDouble().toRadians())
        }

        interpComboBox.addActionListener {
            val newInterpType = interpComboBox.selectedItemTyped ?: return@addActionListener
            if (_waypoint.interpolationType != newInterpType) {
                _waypoint = when (newInterpType) {
                    TrajectoryConfig.HeadingInterpolationType.TANGENT -> {
                        _waypoint.copy(interpolationType = TrajectoryConfig.HeadingInterpolationType.TANGENT, heading = 0.0)
                    }
                    TrajectoryConfig.HeadingInterpolationType.CONSTANT -> {
                        _waypoint.copy(interpolationType = TrajectoryConfig.HeadingInterpolationType.CONSTANT, heading = 0.0)
                    }
                    TrajectoryConfig.HeadingInterpolationType.LINEAR -> {
                        _waypoint.copy(interpolationType = TrajectoryConfig.HeadingInterpolationType.LINEAR, heading = _waypoint.tangent)
                    }
                    TrajectoryConfig.HeadingInterpolationType.SPLINE -> {
                        _waypoint.copy(interpolationType = TrajectoryConfig.HeadingInterpolationType.SPLINE, heading = _waypoint.tangent)
                    }
                }
            }
        }

        tangentTextField.addValueChangeListener<Number> {
            _waypoint = _waypoint.copy(tangent = it.toDouble().toRadians())
        }

        removeButton.addActionListener {
            onWaypointRemove?.invoke()
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

        waypoint = DEFAULT_WAYPOINT
    }
}

data class PathConfig(
    val startPose: Pose2d,
    val startTangent: Double,
    val waypoints: List<TrajectoryConfig.Waypoint>
)

private val DEFAULT_CONFIG = PathConfig(
    Pose2d(),
    0.0,
    emptyList()
)

class PathEditorPanel : JPanel() {
    private val xTextField = makeFormattedDoubleField()
    private val yTextField = makeFormattedDoubleField()
    private val headingTextField = makeFormattedDoubleField()
    private val tangentTextField = makeFormattedDoubleField()
    private val waypointPanelContainer = WidthAgnosticPanel()
    private val waypointPanels = mutableListOf<WaypointPanel>()
    private val headerContainer = JPanel()

    var onConfigChange: ((PathConfig) -> Unit)? = null
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
            if (headingTextField.doubleValue != value.startPose.heading.toDegrees()) {
                headingTextField.value = value.startPose.heading.toDegrees()
            }
            if (tangentTextField.doubleValue != value.startTangent.toDegrees()) {
                tangentTextField.value = value.startTangent.toDegrees()
            }

            // update existing waypoint panels
            for ((i, pair) in value.waypoints.zip(waypointPanels).withIndex()) {
                val (waypoint, panel) = pair
                panel.isVisible = true
                panel.onWaypointRemove = {
                    _config = _config.copy(waypoints = _config.waypoints.filterIndexed { j, _ -> i != j })
                }
                panel.onWaypointChange = { newWaypoint ->
                    _config = _config.copy(waypoints = _config.waypoints.mapIndexed { j, waypoint ->
                        if (i == j) {
                            newWaypoint
                        } else {
                            waypoint
                        }
                    })
                }
                if (panel.waypoint !== waypoint) {
                    panel.waypoint = waypoint
                }
            }

            // create additional panels if necessary
            val startSize = waypointPanels.size
            for ((i, waypoint) in value.waypoints.drop(startSize).withIndex()) {
                val panel = WaypointPanel()
                panel.index = i + startSize
                panel.waypoint = waypoint
                panel.onWaypointRemove = {
                    _config = _config.copy(waypoints = _config.waypoints.filterIndexed { j, _ -> i + startSize != j })
                }
                panel.onWaypointChange = { newWaypoint ->
                    _config = _config.copy(waypoints = _config.waypoints.mapIndexed { j, waypoint ->
                        if (i + startSize == j) {
                            newWaypoint
                        } else {
                            waypoint
                        }
                    })
                }
                waypointPanelContainer.add(panel)
                waypointPanels.add(panel)
            }

            // hide the rest
            for (waypointPanel in waypointPanels.drop(value.waypoints.size)) {
                waypointPanel.isVisible = false
            }

            field = value

            if (!externalUpdate) {
                onConfigChange?.invoke(value)
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
        headingTextField.addValueChangeListener<Number> {
            _config = _config.copy(startPose = _config.startPose.copy(heading = it.toDouble().toRadians()))
        }
        tangentTextField.addValueChangeListener<Number> {
            _config = _config.copy(startTangent = it.toDouble().toRadians())
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
            _config = _config.copy(waypoints = _config.waypoints + listOf(_config.waypoints.lastOrNull()?.copy()
                ?: TrajectoryConfig.Waypoint(_config.startPose.vec(), _config.startPose.heading,
                    _config.startTangent, TrajectoryConfig.HeadingInterpolationType.TANGENT)
            ))
        }
        headerContainer.add(addButton)

        headerContainer.border = BorderFactory.createEmptyBorder(0, 0, 0, 15)
        add(headerContainer, BorderLayout.NORTH)

        val scrollPane = JScrollPane(waypointPanelContainer, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS, JScrollPane.HORIZONTAL_SCROLLBAR_NEVER)
        scrollPane.border = BorderFactory.createEmptyBorder()

        waypointPanelContainer.layout = BoxLayout(waypointPanelContainer, BoxLayout.PAGE_AXIS)

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

        waypointPanelContainer.add(firstRow)

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
