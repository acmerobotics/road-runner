package com.acmerobotics.roadrunner.util

import kotlin.math.ceil
import kotlin.math.floor
import kotlin.math.roundToInt

/**
 * A progression of values of type `Double`.
 */
class DoubleProgression(
    val start: Double,
    val end: Double,
    val step: Double
) : Iterable<Double> {
    private val range: IntRange

    init {
        val steps = ((end - start) / step)
        val diff = steps - floor(steps)
        range = if (diff < step / 1e6) {
            IntRange(0, steps.roundToInt())
        } else {
            IntRange(0, ceil(steps).toInt())
        }
    }

    operator fun plus(offset: Double) =
        DoubleProgression(start + offset, end + offset, step)

    operator fun minus(offset: Double) =
        DoubleProgression(start - offset, end - offset, step)

    operator fun unaryMinus() = DoubleProgression(-start, -end, -step)

    inner class IteratorImpl(range: IntRange) : Iterator<Double> {
        private val iterator: Iterator<Int> = range.iterator()

        override fun hasNext() = iterator.hasNext()

        override fun next() = start + step * iterator.next()
    }

    fun isEmpty() = range.isEmpty()

//     TODO: float equality epsilon?
    override fun equals(other: Any?): Boolean =
            other is DoubleProgression &&
                step == other.step && start == other.start && end == other.end

    override fun hashCode(): Int = range.hashCode() * 31 + step.hashCode()

    override fun iterator() = IteratorImpl(range)

    operator fun contains(query: Double) = query in start..end

    fun split(sep: Double): Pair<DoubleProgression, DoubleProgression> {
        return if (sep in this) {
            val intSep = floor((sep - start) / step)
            DoubleProgression(start, start + intSep * step, step) to
                DoubleProgression(start + (intSep + 1) * step, end, step)
        } else {
            this to DoubleProgression(0.0, -1.0, step)
        }
    }

    override fun toString() = "DoubleProgression[$start, $end, $step]"

    fun items() = range.last - range.first + 1
}

operator fun Double.plus(progression: DoubleProgression) = progression + this

operator fun Double.minus(progression: DoubleProgression) =
    DoubleProgression(this - progression.start, this - progression.end, -progression.step)
