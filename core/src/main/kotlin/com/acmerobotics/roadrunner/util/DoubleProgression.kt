package com.acmerobotics.roadrunner.util

import kotlin.math.ceil
import kotlin.math.floor

/**
 * A progression of values of type `Double`.
 */
class DoubleProgression(
    val start: Double,
    val end: Double,
    val step: Double
): Iterable<Double> {
    private val range: IntRange = IntRange(0, ceil((end - start) / step).toInt())

    operator fun plus(offset: Double) =
        DoubleProgression(start + offset, end + offset, step)

    operator fun minus(offset: Double) =
        DoubleProgression(start - offset, end - offset, step)

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

    fun split(sep: Double): Pair<DoubleProgression, DoubleProgression> {
        val intSep = floor((sep - start) / step)
        return DoubleProgression(start, start + intSep * step, step) to
            DoubleProgression(start + (intSep + 1) * step, end, step)
    }

    override fun toString() = "DoubleProgression[$start, $end, $step]"
}

operator fun Double.plus(progression: DoubleProgression) = progression + this

operator fun Double.minus(progression: DoubleProgression) =
    DoubleProgression(this - progression.start, this - progression.end, progression.step)