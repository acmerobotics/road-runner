package com.acmerobotics.roadrunner.trajectory.config

import com.fasterxml.jackson.core.JsonParser
import com.fasterxml.jackson.databind.DeserializationContext
import com.fasterxml.jackson.databind.JsonNode
import com.fasterxml.jackson.databind.deser.std.StdDeserializer

class TrajectoryConfigDeserializer : StdDeserializer<TrajectoryConfig>(TrajectoryConfig::class.java) {
    override fun deserialize(p: JsonParser?, ctxt: DeserializationContext?): TrajectoryConfig? {
        p ?: return null
        val node = p.readValueAsTree<JsonNode>()
        return if (node.has("version")) {
            val version = node["version"].intValue()
            if (version == 2) {
                p.codec.treeToValue(node, TrajectoryConfig::class.java)
            } else {
                null
            }
        } else {
            // legacy V1
            p.codec.treeToValue(node, LegacyTrajectoryConfig::class.java).toTrajectoryConfig()
        }
    }
}
