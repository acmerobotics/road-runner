package com.acmerobotics.roadrunner

// Ported from https://github.com/janestreet/sexp_pretty/blob/9ed833044944eb9d803be9a0753ccd15990cd387/src/sexp_pretty.ml

// The MIT License
//
// Copyright (c) 2016--2024 Jane Street Group, LLC <opensource-contacts@janestreet.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

import java.io.Writer

/**
 * S-expression representation.
 */
sealed class Sexp {
    /**
     * Atom represents a terminal value in the S-expression tree.
     */
    data class Atom(val value: String) : Sexp() {
        override fun toString(): String = value
    }

    /**
     * List represents a sequence of S-expressions.
     */
    data class List(val elements: kotlin.collections.List<Sexp>) : Sexp() {
        constructor(vararg elements: Sexp) : this(elements.toList())

        override fun toString(): String =
            "(${elements.joinToString(" ")})"
    }

    companion object {
        /**
         * Create an atom from a string.
         */
        fun atom(value: String): Atom = Atom(value)

        /**
         * Create a list from multiple S-expressions.
         */
        fun list(vararg elements: Sexp): List = List(*elements)

        /**
         * Create a list from a collection of S-expressions.
         */
        fun list(elements: kotlin.collections.List<Sexp>): List = List(elements)

        /**
         * Parse a string into an S-expression.
         */
        fun parse(input: String): Sexp {
            val parser = SexpParser(input)
            return parser.parse()
        }
    }
}

/**
 * A simple S-expression parser.
 */
private class SexpParser(private val input: String) {
    private var position = 0

    fun parse(): Sexp {
        skipWhitespace()
        return parseExpression()
    }

    private fun parseExpression(): Sexp {
        if (position >= input.length) {
            throw IllegalArgumentException("Unexpected end of input")
        }

        return when (val c = input[position]) {
            '(' -> parseList()
            else -> parseAtom()
        }
    }

    private fun parseList(): Sexp {
        // Skip '('
        position++

        val elements = mutableListOf<Sexp>()
        skipWhitespace()

        while (position < input.length && input[position] != ')') {
            elements.add(parseExpression())
            skipWhitespace()
        }

        if (position >= input.length || input[position] != ')') {
            throw IllegalArgumentException("Expected closing parenthesis")
        }

        // Skip ')'
        position++

        return Sexp.List(elements)
    }

    private fun parseAtom(): Sexp {
        val start = position

        while (position < input.length && !isDelimiter(input[position])) {
            position++
        }

        if (start == position) {
            throw IllegalArgumentException("Empty atom at position $position")
        }

        return Sexp.Atom(input.substring(start, position))
    }

    private fun isDelimiter(c: Char): Boolean =
        c.isWhitespace() || c == '(' || c == ')'

    private fun skipWhitespace() {
        while (position < input.length && input[position].isWhitespace()) {
            position++
        }
    }
}

/**
 * Color definitions for terminal output.
 */
enum class Color(val code: Int) {
    BLACK(30),
    RED(31),
    GREEN(32),
    YELLOW(33),
    BLUE(34),
    MAGENTA(35),
    CYAN(36),
    WHITE(37),
    BRIGHT_BLACK(90),
    BRIGHT_RED(91),
    BRIGHT_GREEN(92),
    BRIGHT_YELLOW(93),
    BRIGHT_BLUE(94),
    BRIGHT_MAGENTA(95),
    BRIGHT_CYAN(96),
    BRIGHT_WHITE(97),
    DEFAULT(39);
}

/**
 * Comment handling mode.
 */
sealed class CommentHandling {
    /**
     * Drop all comments.
     */
    object Drop : CommentHandling()

    /**
     * Print comments with the given options.
     */
    data class Print(
        val indent: CommentIndent,
        val color: Color?,
        val style: CommentStyle
    ) : CommentHandling()
}

/**
 * Comment indentation mode.
 */
sealed class CommentIndent {
    /**
     * Auto-indent comments.
     */
    object AutoIndent : CommentIndent()

    /**
     * Indent comments with a fixed number of spaces.
     */
    data class Fixed(val spaces: Int) : CommentIndent()
}

/**
 * Comment style.
 */
enum class CommentStyle {
    /**
     * Print comments as they are.
     */
    CONSERVATIVE_PRINT,

    /**
     * Pretty-print comments.
     */
    PRETTY_PRINT
}

/**
 * Atom coloring mode.
 */
sealed class AtomColoring {
    /**
     * Don't color atoms.
     */
    object None : AtomColoring()

    /**
     * Color the first atom if it's length is less than or equal to the threshold.
     */
    data class First(val threshold: Int) : AtomColoring()

    /**
     * Color all atoms.
     */
    object All : AtomColoring()
}

/**
 * Atom printing mode.
 */
enum class AtomPrinting {
    /**
     * Escape special characters.
     */
    ESCAPED,

    /**
     * Interpret atom contents.
     */
    INTERPRETED,

    /**
     * Minimal escaping of atom contents.
     */
    MINIMAL_ESCAPING
}

/**
 * Position of opening parentheses.
 */
enum class OpeningParens {
    /**
     * Put opening parentheses on a new line.
     */
    NEW_LINE,

    /**
     * Keep opening parentheses on the same line.
     */
    SAME_LINE
}

/**
 * Position of closing parentheses.
 */
enum class ClosingParens {
    /**
     * Put closing parentheses on a new line.
     */
    NEW_LINE,

    /**
     * Keep closing parentheses on the same line.
     */
    SAME_LINE
}

/**
 * Separator between top-level S-expressions.
 */
enum class Separator {
    /**
     * No separator.
     */
    NONE,

    /**
     * Empty line separator.
     */
    EMPTY_LINE
}

/**
 * Data alignment mode.
 */
sealed class DataAlignment {
    /**
     * Don't align data.
     */
    object NotAligned : DataAlignment()

    /**
     * Align data with the given options.
     */
    data class Aligned(
        val parensAlignment: ParensAlignment,
        val atomThreshold: Int,
        val charThreshold: Int,
        val depthThreshold: Int
    ) : DataAlignment()
}

/**
 * Parentheses alignment mode.
 */
enum class ParensAlignment {
    /**
     * Align parentheses.
     */
    ALIGN,

    /**
     * Don't align parentheses.
     */
    DONT_ALIGN
}

/**
 * Comment placement mode.
 */
enum class StickyComments {
    /**
     * Put comments on the same line as the expression they're attached to.
     */
    SAME_LINE,

    /**
     * Put comments before the expression they're attached to.
     */
    BEFORE,

    /**
     * Put comments after the expression they're attached to.
     */
    AFTER
}

/**
 * Configuration for S-expression pretty-printing.
 */
data class Config(
    val indent: Int = 2,
    val colorScheme: List<Color> = listOf(
        Color.RED, Color.GREEN, Color.YELLOW, Color.BLUE, Color.MAGENTA, Color.CYAN
    ),
    val parenColoring: Boolean = true,
    val atomColoring: AtomColoring = AtomColoring.None,
    val atomPrinting: AtomPrinting = AtomPrinting.ESCAPED,
    val comments: CommentHandling = CommentHandling.Print(
        CommentIndent.AutoIndent,
        Color.BRIGHT_BLACK,
        CommentStyle.PRETTY_PRINT
    ),
    val openingParens: OpeningParens = OpeningParens.SAME_LINE,
    val closingParens: ClosingParens = ClosingParens.SAME_LINE,
    val separator: Separator = Separator.EMPTY_LINE,
    val dataAlignment: DataAlignment = DataAlignment.NotAligned,
    val stickyComments: StickyComments = StickyComments.SAME_LINE,
    val singletonLimit: Pair<Int, Int> = Pair(3, 50), // Atom threshold, char threshold
    val leadingThreshold: Pair<Int, Int> = Pair(3, 30) // Atom threshold, char threshold
) {
    companion object {
        /**
         * Create a configuration with the given color mode.
         */
        fun create(color: Boolean = true): Config {
            return if (color) {
                Config()
            } else {
                Config(
                    parenColoring = false,
                    atomColoring = AtomColoring.None,
                    comments = CommentHandling.Print(
                        CommentIndent.AutoIndent,
                        null,
                        CommentStyle.PRETTY_PRINT
                    )
                )
            }
        }
    }
}

/**
 * Represents a formatted line in the pretty-printed output.
 */
private sealed class FormattedLine {
    /**
     * A line containing an atom or list.
     */
    data class Expression(val indent: Int, val content: String) : FormattedLine()

    /**
     * A line containing a comment.
     */
    data class Comment(val indent: Int, val content: String) : FormattedLine()
}

/**
 * Tracks the state of the pretty-printer.
 */
private data class PrinterState(
    val depth: Int = 0,
    val currentLine: StringBuilder = StringBuilder(),
    val lines: MutableList<FormattedLine> = mutableListOf(),
    val inComment: Boolean = false
)

/**
 * Main pretty-printer class.
 */
class SexpPrettyPrinter(private val config: Config = Config()) {

    /**
     * Pretty-print an S-expression to a string.
     */
    fun prettyPrint(sexp: Sexp): String {
        val state = PrinterState()
        prettyPrintInternal(sexp, state)
        flushLine(state)
        return state.lines.joinToString("\n") { formattedLine ->
            when (formattedLine) {
                is FormattedLine.Expression ->
                    " ".repeat(formattedLine.indent) + formattedLine.content
                is FormattedLine.Comment -> {
                    val prefix = if (config.comments is CommentHandling.Print &&
                        config.comments.color != null
                    ) {
                        "\u001B[${config.comments.color.code}m"
                    } else {
                        ""
                    }
                    val suffix = if (config.comments is CommentHandling.Print &&
                        config.comments.color != null
                    ) {
                        "\u001B[0m"
                    } else {
                        ""
                    }
                    " ".repeat(formattedLine.indent) + prefix + formattedLine.content + suffix
                }
            }
        }
    }

    /**
     * Pretty-print an S-expression to a writer.
     */
    fun prettyPrint(sexp: Sexp, writer: Writer) {
        writer.write(prettyPrint(sexp))
    }

    private fun prettyPrintInternal(sexp: Sexp, state: PrinterState) {
        when (sexp) {
            is Sexp.Atom -> printAtom(sexp, state)
            is Sexp.List -> printList(sexp, state)
        }
    }

    private fun printAtom(atom: Sexp.Atom, state: PrinterState) {
        val value = when (config.atomPrinting) {
            AtomPrinting.ESCAPED -> escapeString(atom.value)
            AtomPrinting.INTERPRETED -> atom.value
            AtomPrinting.MINIMAL_ESCAPING ->
                if (mustEscape(atom.value)) minimalEscaping(atom.value) else atom.value
        }

        val colored = when (config.atomColoring) {
            is AtomColoring.None -> value
            is AtomColoring.First ->
                if (state.depth == 0 && value.length <= config.atomColoring.threshold) {
                    colorString(value, state.depth)
                } else {
                    value
                }
            is AtomColoring.All -> colorString(value, state.depth)
        }

        state.currentLine.append(colored)
    }

    private fun printList(list: Sexp.List, state: PrinterState) {
        if (list.elements.isEmpty()) {
            printEmptyList(state)
            return
        }

        // Check if this list can be printed as a singleton
        if (tryPrintAsSingleton(list, state)) {
            return
        }

        // Try to print on a single line if possible
        if (canPrintOnSingleLine(list)) {
            printListOnSingleLine(list, state)
            return
        }

        // Otherwise print with indentation
        printListWithIndentation(list, state)
    }

    private fun printEmptyList(state: PrinterState) {
        val openParen = if (config.parenColoring) {
            colorString("(", state.depth)
        } else {
            "("
        }

        val closeParen = if (config.parenColoring) {
            colorString(")", state.depth)
        } else {
            ")"
        }

        state.currentLine.append(openParen).append(closeParen)
    }

    private fun tryPrintAsSingleton(list: Sexp.List, state: PrinterState): Boolean {
        val (atomThreshold, charThreshold) = config.singletonLimit

        // Check if the list has the structure (atom1 atom2 ... (sublist))
        val atoms = mutableListOf<Sexp.Atom>()
        var lastElement: Sexp? = null

        for (elem in list.elements) {
            if (elem is Sexp.Atom && atoms.size < atomThreshold) {
                atoms.add(elem)
            } else {
                lastElement = elem
                break
            }
        }

        if (atoms.size < atomThreshold &&
            lastElement is Sexp.List &&
            list.elements.size == atoms.size + 1
        ) {

            // Check character count
            val totalCharCount = atoms.sumOf { it.value.length }
            if (totalCharCount <= charThreshold) {
                // Print as singleton
                val openParen = if (config.parenColoring) {
                    colorString("(", state.depth)
                } else {
                    "("
                }

                state.currentLine.append(openParen)

                // Print leading atoms
                atoms.forEachIndexed { index, atom ->
                    if (index > 0) {
                        state.currentLine.append(" ")
                    }
                    printAtom(atom, state)
                }

                // Print next list with increased depth
                state.currentLine.append(" ")
                val newState = state.copy(depth = state.depth + 1)
                prettyPrintInternal(lastElement, newState)

                val closeParen = if (config.parenColoring) {
                    colorString(")", state.depth)
                } else {
                    ")"
                }

                state.currentLine.append(closeParen)
                return true
            }
        }

        return false
    }

    private fun canPrintOnSingleLine(list: Sexp.List): Boolean {
        // Simple heuristic: can print on a single line if the total length is less than 80 chars
        val totalLength = estimateLength(list)
        return totalLength < 80
    }

    private fun estimateLength(sexp: Sexp): Int {
        return when (sexp) {
            is Sexp.Atom -> sexp.value.length
            is Sexp.List -> {
                // 2 for the parentheses
                var length = 2
                // Add length of each element plus a space
                for (elem in sexp.elements) {
                    length += estimateLength(elem) + 1
                }
                // Subtract the extra space after the last element
                if (sexp.elements.isNotEmpty()) {
                    length -= 1
                }
                length
            }
        }
    }

    private fun printListOnSingleLine(list: Sexp.List, state: PrinterState) {
        val openParen = if (config.parenColoring) {
            colorString("(", state.depth)
        } else {
            "("
        }

        state.currentLine.append(openParen)

        list.elements.forEachIndexed { index, element ->
            if (index > 0) {
                state.currentLine.append(" ")
            }

            prettyPrintInternal(element, state.copy(depth = state.depth + 1))
        }

        val closeParen = if (config.parenColoring) {
            colorString(")", state.depth)
        } else {
            ")"
        }

        state.currentLine.append(closeParen)
    }

    private fun printListWithIndentation(list: Sexp.List, state: PrinterState) {
        val openParen = if (config.parenColoring) {
            colorString("(", state.depth)
        } else {
            "("
        }

        // Print opening paren
        state.currentLine.append(openParen)

        // For first element, keep it on the same line if opening parens are on the same line
        val firstElement = list.elements.firstOrNull()
        if (firstElement != null && config.openingParens == OpeningParens.SAME_LINE) {
            state.currentLine.append(" ")
            prettyPrintInternal(firstElement, state.copy(depth = state.depth + 1))
            flushLine(state)
        } else if (firstElement != null) {
            flushLine(state)
            // Indent for the first element
            state.currentLine.append(" ".repeat(config.indent))
            prettyPrintInternal(firstElement, state.copy(depth = state.depth + 1))
            flushLine(state)
        }

        // Print remaining elements with proper indentation
        for (i in 1 until list.elements.size) {
            state.currentLine.append(" ".repeat(config.indent))
            prettyPrintInternal(list.elements[i], state.copy(depth = state.depth + 1))
            flushLine(state)
        }

        // Print closing paren
        val closeParen = if (config.parenColoring) {
            colorString(")", state.depth)
        } else {
            ")"
        }

        if (config.closingParens == ClosingParens.SAME_LINE) {
            state.currentLine.append(closeParen)
        } else {
            flushLine(state)
            state.currentLine.append(closeParen)
        }
    }

    private fun flushLine(state: PrinterState) {
        if (state.currentLine.isNotEmpty()) {
            val line = if (state.inComment) {
                FormattedLine.Comment(state.depth * config.indent, state.currentLine.toString())
            } else {
                FormattedLine.Expression(state.depth * config.indent, state.currentLine.toString())
            }
            state.lines.add(line)
            state.currentLine.clear()
        }
    }

    private fun colorString(str: String, depth: Int): String {
        if (!config.parenColoring) return str

        val colorIndex = depth % config.colorScheme.size
        val color = config.colorScheme[colorIndex]
        return "\u001B[${color.code}m$str\u001B[0m"
    }

    private fun mustEscape(str: String): Boolean {
        if (str == "\\") return false

        // Check if the string needs to be escaped
        return str.isEmpty() ||
            str.any { it.isWhitespace() || it == '(' || it == ')' || it == '"' || !it.isPrintable() }
    }

    private fun Char.isPrintable(): Boolean {
        return this.toInt() in 32..126
    }

    private fun escapeString(str: String): String {
        val sb = StringBuilder()
        for (c in str) {
            when (c) {
                '"', '\\' -> sb.append('\\').append(c)
                '\n' -> sb.append("\\n")
                '\t' -> sb.append("\\t")
                '\r' -> sb.append("\\r")
                else -> if (c.isPrintable()) sb.append(c) else
                    sb.append("\\u${c.code.toString(16).padStart(4, '0')}")
            }
        }
        return if (mustEscape(str)) "\"$sb\"" else sb.toString()
    }

    private fun minimalEscaping(str: String): String {
        val sb = StringBuilder("\"")
        for (c in str) {
            when (c) {
                '"', '\\' -> sb.append('\\').append(c)
                ' ', '\t', '\n' -> sb.append(c)
                else -> if (c.isPrintable()) sb.append(c) else
                    sb.append("\\u${c.code.toString(16).padStart(4, '0')}")
            }
        }
        sb.append("\"")
        return sb.toString()
    }
}

/**
 * Main entry point for pretty-printing S-expressions.
 */
fun prettyPrint(sexp: Sexp, config: Config = Config()): String {
    val printer = SexpPrettyPrinter(config)
    return printer.prettyPrint(sexp)
}

/**
 * Main entry point for pretty-printing S-expressions to a writer.
 */
fun prettyPrint(sexp: Sexp, writer: Writer, config: Config = Config()) {
    val printer = SexpPrettyPrinter(config)
    printer.prettyPrint(sexp, writer)
}

/**
 * Format an S-expression string.
 */
fun prettyPrintString(input: String, config: Config = Config()): String {
    val sexp = Sexp.parse(input)
    return prettyPrint(sexp, config)
}

/**
 * Pretty-print an S-expression to a string with colors disabled.
 */
fun sexpToString(sexp: Sexp): String {
    val config = Config.create(color = false)
    return prettyPrint(sexp, config)
}

// Example usage
fun main() {
    val sexp = Sexp.List(
        Sexp.Atom("define"),
        Sexp.Atom("factorial"),
        Sexp.List(
            Sexp.Atom("lambda"),
            Sexp.List(Sexp.Atom("n")),
            Sexp.List(
                Sexp.Atom("if"),
                Sexp.List(
                    Sexp.Atom("="),
                    Sexp.Atom("n"),
                    Sexp.Atom("0")
                ),
                Sexp.Atom("1"),
                Sexp.List(
                    Sexp.Atom("*"),
                    Sexp.Atom("n"),
                    Sexp.List(
                        Sexp.Atom("factorial"),
                        Sexp.List(
                            Sexp.Atom("-"),
                            Sexp.Atom("n"),
                            Sexp.Atom("1")
                        )
                    )
                )
            )
        )
    )

    // Print with default config
    println("Default formatting:")
    println(prettyPrint(sexp))

    // Print with custom config
    println("\nCustom formatting:")
    val customConfig = Config(
        indent = 2,
        atomColoring = AtomColoring.First(10),
        openingParens = OpeningParens.NEW_LINE,
        closingParens = ClosingParens.NEW_LINE
    )
    println(prettyPrint(sexp, customConfig))

    // Parse and pretty-print a string
    println("\nParsed from string:")
    val parsedSexp = Sexp.parse("(hello (world) (this is (a nested) s-expression))")
    println(prettyPrint(parsedSexp))
}
