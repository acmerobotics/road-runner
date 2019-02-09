package com.acmerobotics.roadrunner.localization

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.DecompositionSolver
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils
import kotlin.math.cos
import kotlin.math.sin

/**
 * Localizer based on three unpowered tracking omni wheels.
 *
 * @param wheelPositions wheel positions relative to the center of the robot (positive X points forward on the robot)
 * @param wheelOrientations angular orientations of the wheels measured counterclockwise from positive X in radians
 */
// TODO: do localizers belong here or in their own package?
abstract class ThreeTrackingWheelLocalizer(
    wheelPositions: List<Vector2d>,
    wheelOrientations: List<Double>
) : Localizer {
    override var poseEstimate: Pose2d = Pose2d()
        set(value) {
            lastWheelPositions = emptyList()
            field = value
        }
    private var lastWheelPositions = emptyList<Double>()

    private val forwardSolver: DecompositionSolver

    init {
        if (wheelPositions.size != 3) {
            throw IllegalArgumentException("3 wheel positions must be provided")
        }

        if (wheelOrientations.size != 3) {
            throw IllegalArgumentException("3 wheel orientations must be provided")
        }

        val inverseMatrix = Array2DRowRealMatrix(3, 3)
        for (i in 0..2) {
            val orientationVector =
                Vector2d(cos(wheelOrientations[i]), sin(wheelOrientations[i]))
            val positionVector = wheelPositions[i]
            inverseMatrix.setEntry(i, 0, orientationVector.x)
            inverseMatrix.setEntry(i, 1, orientationVector.y)
            inverseMatrix.setEntry(i, 2,
                    positionVector.x * orientationVector.y - positionVector.y * orientationVector.x)
        }

        forwardSolver = LUDecomposition(inverseMatrix).solver

        if (!forwardSolver.isNonSingular) {
            throw IllegalArgumentException("The specified wheel positions and orientations are not sufficient for full localization")
        }
    }

    override fun update() {
        val wheelPositions = getWheelPositions()
        if (lastWheelPositions.isNotEmpty()) {
            val wheelDeltas = wheelPositions
                    .zip(lastWheelPositions)
                    .map { it.first - it.second }
            val rawPoseDelta = forwardSolver.solve(MatrixUtils.createRealMatrix(
                    arrayOf(wheelDeltas.toDoubleArray())
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
    }

    /**
     * Returns the positions of the tracking wheels in the desired distance units (not encoder counts!)
     */
    abstract fun getWheelPositions(): List<Double>
}