package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.*

/**
 * Projection utility that supports both localized and global projection applied along an entire [Path]
 *
 * @param path the path to be projected along
 * @param thresholdDeltaTangent the threshold tangent angle difference (should be < PI / 2 for good results) for a point
 * to be added to initial guesses list
 * @param maxDeltaS maximum spacing between points evaluated for angle check
 * @param maxDeltaTangent the maximum difference in tangent angle allowed between points evaluated for angle check
 * @param maxDepth the maximum recursive depth allowed for the subdividing check
 */
class PathProjector(
    private val path: Path,
    private val thresholdDeltaTangent: Double = PI / 4,
    private val maxDeltaS: Double = 0.25,
    private val maxDeltaTangent: Double = PI / 16,
    private val maxDepth: Int = 30
) {

    private val guesses = mutableListOf(0.0).apply {
        val cosThresholdDTangent = cos(thresholdDeltaTangent)
        val cosMaxDTangent = cos(maxDeltaTangent)

        var lastGuessTangent = path.deriv(0.0).vec()

        fun subdivide(
            sLo: Double,
            sHi: Double,
            vLo: Vector2d = path[sLo].vec(),
            vHi: Vector2d = path[sHi].vec(),
            depth: Int = 0
        ) {
            val sMid = 0.5 * (sLo + sHi)
            val vMid = path[sMid].vec()

            val tangentHi = path.deriv(sHi).vec()
            val tangentLo = path.deriv(sLo).vec()

            if ((sHi - sLo > maxDeltaS || tangentHi dot tangentLo < cosMaxDTangent) && depth < maxDepth) {
                subdivide(sLo, sMid, vLo, vMid, depth + 1)
                subdivide(sMid, sHi, vMid, vHi, depth + 1)
            } else if (tangentLo dot lastGuessTangent < cosThresholdDTangent) {
                add(sLo)
                lastGuessTangent = tangentLo
            }
        }

        subdivide(0.0, path.length())
    }

    /**
     * Project [queryPoint] onto the current path using the iterative method described
     * [here](http://www.geometrie.tugraz.at/wallner/sproj.pdf).
     *
     * @param queryPoint query queryPoint
     * @param projectGuess guess for the projected queryPoint's s along the path
     */
    fun localProject(queryPoint: Vector2d, projectGuess: Double = path.length() / 2.0): Double {
        // we use the first-order method (since we already compute the arc length param)
        var s = projectGuess
        run {
            repeat(200) {
                val t = path.reparam(s)
                val pathPoint = path[s, t].vec()
                val deriv = path.deriv(s, t).vec()

                val ds = (queryPoint - pathPoint) dot deriv

                if (ds epsilonEquals 0.0) {
                    return@run
                }

                s += ds

                if (s <= 0.0) {
                    return@run
                }

                if (s >= path.length()) {
                    return@run
                }
            }
        }

        return max(0.0, min(s, path.length()))
    }

    /**
     * Project [queryPoint] onto the current path by applying [localProject] with various
     * guesses along the path.
     *
     * @param queryPoint query queryPoint
     */
    fun project(queryPoint: Vector2d): Double {
        val results = guesses.map { localProject(queryPoint, it) }

        return results.minBy { path[it].vec().distTo(queryPoint) } ?: 0.0
    }
}
