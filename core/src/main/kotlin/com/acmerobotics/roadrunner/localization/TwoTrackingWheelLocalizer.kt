package com.acmerobotics.roadrunner.localization

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import kotlin.math.cos
import kotlin.math.sin
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.DecompositionSolver
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils

/**
 * Localizer based on two unpowered tracking omni wheels and an orientation sensor.
 *
 * @param wheelPositions wheel positions relative to the center of the robot (positive X points forward on the robot)
 * @param wheelOrientations angular orientations of the wheels measured counterclockwise from positive X in radians
 */
abstract class TwoTrackingWheelLocalizer(
    wheelPositions: List<Vector2d>,
    wheelOrientations: List<Double>
) : Localizer {
    override var poseEstimate: Pose2d =
        Pose2d()
        set(value) {
            lastWheelPositions = emptyList()
            lastHeading = Double.NaN
            field = value
        }
    private var lastWheelPositions = emptyList<Double>()
    private var lastHeading = Double.NaN

    private val forwardSolver: DecompositionSolver

    init {
        if (wheelPositions.size != 2) {
            throw IllegalArgumentException("2 wheel positions must be provided")
        }

        if (wheelOrientations.size != 2) {
            throw IllegalArgumentException("2 wheel orientations must be provided")
        }

        val inverseMatrix = Array2DRowRealMatrix(3, 3)
        for (i in 0..1) {
            val orientationVector =
                Vector2d(cos(wheelOrientations[i]), sin(wheelOrientations[i]))
            val positionVector = wheelPositions[i]
            inverseMatrix.setEntry(i, 0, orientationVector.x)
            inverseMatrix.setEntry(i, 1, orientationVector.y)
            inverseMatrix.setEntry(i, 2,
                    positionVector.x * orientationVector.y - positionVector.y * orientationVector.x)
        }
        inverseMatrix.setEntry(2, 2, 1.0)

        forwardSolver = LUDecomposition(inverseMatrix).solver

        if (!forwardSolver.isNonSingular) {
            throw IllegalArgumentException("The specified configuration cannot support full localization")
        }
    }

    override fun update() {
        val wheelPositions = getWheelPositions()
        val heading = getHeading()
        if (lastWheelPositions.isNotEmpty()) {
            val wheelDeltas = wheelPositions
                    .zip(lastWheelPositions)
                    .map { it.first - it.second }
            val headingDelta = heading - lastHeading
            val rawPoseDelta = forwardSolver.solve(MatrixUtils.createRealMatrix(
                    arrayOf((wheelDeltas + headingDelta).toDoubleArray())
            ).transpose())
            val robotPoseDelta = Pose2d(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0)
            )
            poseEstimate =
                Kinematics.relativeOdometryUpdate(poseEstimate, robotPoseDelta)
        }
        lastWheelPositions = wheelPositions
        lastHeading = heading
    }

    /**
     * Returns the positions of the tracking wheels in the desired distance units (not encoder counts!)
     */
    abstract fun getWheelPositions(): List<Double>

    /**
     * Returns the heading of the robot (usually from a gyroscope or IMU).
     */
    abstract fun getHeading(): Double
}
