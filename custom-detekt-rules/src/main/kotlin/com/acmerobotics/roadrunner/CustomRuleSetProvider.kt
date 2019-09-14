package com.acmerobotics.roadrunner

import io.gitlab.arturbosch.detekt.api.Config
import io.gitlab.arturbosch.detekt.api.RuleSet
import io.gitlab.arturbosch.detekt.api.RuleSetProvider

/**
 * Project-specific rule set provider.
 */
class CustomRuleSetProvider : RuleSetProvider {
    override val ruleSetId: String = "custom-rules"

    override fun instance(config: Config) =
        RuleSet(ruleSetId, listOf(NoJavaMath(config)))
}
