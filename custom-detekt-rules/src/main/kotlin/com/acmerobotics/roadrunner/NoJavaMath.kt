package com.acmerobotics.roadrunner

import io.gitlab.arturbosch.detekt.api.*
import org.jetbrains.kotlin.psi.KtDotQualifiedExpression
import org.jetbrains.kotlin.psi.KtImportDirective

/**
 * Rule to encourage the use of kotlin.math.* functions in place of the java.lang.Math versions.
 */
class NoJavaMath(
    config: Config = Config.empty
) : Rule(config) {
    companion object {
        const val IGNORE = "ignore"
    }

    override val issue = Issue(javaClass.simpleName,
        Severity.Maintainability, "Prefer Kotlin to Java math.", Debt.FIVE_MINS)

    private val ignore: List<String> =
        valueOrDefault(IGNORE, "")
            .split(",")
            .filter { it.isNotBlank() }

    override fun visitImportDirective(importDirective: KtImportDirective) {
        if ("java.lang.Math" in importDirective.text) {
            report(CodeSmell(issue, Entity.from(importDirective), "Suspicious java.lang.Math import"))
        }
    }

    override fun visitDotQualifiedExpression(expression: KtDotQualifiedExpression) {
        if (expression.text.startsWith("Math.") &&
            expression.text.removeSurrounding(".").split("(").first() !in ignore) {
            report(CodeSmell(issue, Entity.from(expression), "java.lang.Math qualified expression"))
        }
    }
}
