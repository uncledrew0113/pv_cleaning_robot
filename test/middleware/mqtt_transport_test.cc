#include <catch2/catch.hpp>

#include "pv_cleaning_robot/middleware/mqtt_transport.h"

using robot::middleware::detail::mqtt_topic_matches;

TEST_CASE("MqttTopicMatch: exact topic matches", "[middleware][mqtt]") {
    REQUIRE(mqtt_topic_matches("a/b/c", "a/b/c"));
    REQUIRE_FALSE(mqtt_topic_matches("a/b/c", "a/b/d"));
}

TEST_CASE("MqttTopicMatch: '+' single-level wildcard", "[middleware][mqtt]") {
    REQUIRE(mqtt_topic_matches("v1/devices/me/rpc/request/+", "v1/devices/me/rpc/request/123"));
    REQUIRE_FALSE(mqtt_topic_matches("v1/+/me", "v1/a/b/me"));
}

TEST_CASE("MqttTopicMatch: '#' multi-level wildcard", "[middleware][mqtt]") {
    REQUIRE(mqtt_topic_matches("a/#", "a/b/c"));
    REQUIRE(mqtt_topic_matches("a/#", "a"));
    REQUIRE_FALSE(mqtt_topic_matches("a/#/x", "a/b/x"));
}
