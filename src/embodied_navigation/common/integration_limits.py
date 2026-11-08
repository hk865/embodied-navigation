"""Frozen integration limits, named by quantity and time basis.

Names with equal values remain separate when their semantics differ.
These are the existing limits, not new profile controls.
"""
MATCH_CORRECTION_MAX_AGE_NS = 3_000_000_000
SOURCE_JOIN_SLOT_MAX_AGE_NS = 3_000_000_000
KNOWN_BUNDLE_FRESHNESS_NS = 300_000_000
KNOWN_BUNDLE_FRESHNESS_S = KNOWN_BUNDLE_FRESHNESS_NS / 1_000_000_000
COMMAND_SOURCE_FRESHNESS_NS = 300_000_000
COMMAND_SOURCE_FRESHNESS_S = COMMAND_SOURCE_FRESHNESS_NS / 1_000_000_000
RAY_ABS_P95_LIMIT_M = .35
LIDAR_BODY_TRANSLATION_M = (.2, 0., .1177)
