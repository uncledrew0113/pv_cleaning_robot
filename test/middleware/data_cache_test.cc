/**
 * DataCache 中间件单元测试
 * [middleware][data_cache]
 */
#include <catch2/catch.hpp>
#include <filesystem>
#include <fstream>
#include <rapidjson/document.h>
#include <string>

#include "pv_cleaning_robot/middleware/data_cache.h"

using robot::middleware::DataCache;
namespace fs = std::filesystem;

// ────────────────────────────────────────────────────────────────
// 使用临时目录，测试后自动清理
// ────────────────────────────────────────────────────────────────
struct CacheFixture {
    std::string path{"/tmp/test_data_cache_" +
                     std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
                     ".jsonl"};
    DataCache cache;

    CacheFixture() : cache(path) {
        cache.open();
    }
    ~CacheFixture() {
        cache.close();
        fs::remove(path);
        fs::remove(path + ".tmp");
    }
};

// ────────────────────────────────────────────────────────────────
// open() / close()
// ────────────────────────────────────────────────────────────────
TEST_CASE("DataCache: open() 创建父目录并返回 true", "[middleware][data_cache]") {
    std::string p = "/tmp/datacache_test_subdir/test.jsonl";
    DataCache cache(p);
    REQUIRE(cache.open());
    cache.close();
    fs::remove(p);
    fs::remove_all("/tmp/datacache_test_subdir");
}

// ────────────────────────────────────────────────────────────────
// push() / size()
// ────────────────────────────────────────────────────────────────
TEST_CASE("DataCache: push() 后 size() 增加", "[middleware][data_cache]") {
    CacheFixture f;
    REQUIRE(f.cache.size() == 0);
    REQUIRE(f.cache.push("topic/t", "payload1"));
    REQUIRE(f.cache.size() == 1);
    REQUIRE(f.cache.push("topic/t", "payload2"));
    REQUIRE(f.cache.size() == 2);
}

TEST_CASE("DataCache: push() 空 topic 和 payload 也可写入", "[middleware][data_cache]") {
    CacheFixture f;
    REQUIRE(f.cache.push("", ""));
    REQUIRE(f.cache.size() == 1);
}

// ────────────────────────────────────────────────────────────────
// pop_batch()
// ────────────────────────────────────────────────────────────────
TEST_CASE("DataCache: pop_batch() 返回正确条数且不删除", "[middleware][data_cache]") {
    CacheFixture f;
    f.cache.push("a", "1");
    f.cache.push("b", "2");
    f.cache.push("c", "3");

    auto batch = f.cache.pop_batch(2);
    REQUIRE(batch.size() == 2);
    REQUIRE(f.cache.size() == 3);  // pop_batch 不删除
}

TEST_CASE("DataCache: pop_batch() 内容与 push() 顺序一致", "[middleware][data_cache]") {
    CacheFixture f;
    f.cache.push("topic1", "data1", 1000);
    f.cache.push("topic2", "data2", 2000);

    auto batch = f.cache.pop_batch(10);
    REQUIRE(batch.size() == 2);
    REQUIRE(batch[0].topic == "topic1");
    REQUIRE(batch[0].payload == "data1");
    REQUIRE(batch[1].topic == "topic2");
}

TEST_CASE("DataCache: pop_batch() 在空缓存时返回空向量", "[middleware][data_cache]") {
    CacheFixture f;
    auto batch = f.cache.pop_batch(10);
    REQUIRE(batch.empty());
}

// ────────────────────────────────────────────────────────────────
// confirm_sent()
// ────────────────────────────────────────────────────────────────
TEST_CASE("DataCache: confirm_sent() 删除指定 id 的记录", "[middleware][data_cache]") {
    CacheFixture f;
    f.cache.push("t", "a");
    f.cache.push("t", "b");
    f.cache.push("t", "c");

    auto batch = f.cache.pop_batch(10);
    REQUIRE(batch.size() == 3);

    // 确认前两条已发送
    f.cache.confirm_sent({batch[0].id, batch[1].id});
    REQUIRE(f.cache.size() == 1);

    // 剩余条目应是第3条
    auto remaining = f.cache.pop_batch(10);
    REQUIRE(remaining.size() == 1);
    REQUIRE(remaining[0].payload == "c");
}

TEST_CASE("DataCache: confirm_sent() 空 id 列表不崩溃", "[middleware][data_cache]") {
    CacheFixture f;
    f.cache.push("t", "x");
    REQUIRE_NOTHROW(f.cache.confirm_sent({}));
    REQUIRE(f.cache.size() == 1);
}

// ────────────────────────────────────────────────────────────────
// max_rows 上限：超出时丢弃最旧记录
// ────────────────────────────────────────────────────────────────
TEST_CASE("DataCache: 超出 max_rows 时丢弃最旧记录", "[middleware][data_cache]") {
    std::string path = "/tmp/test_dc_maxrows.jsonl";
    DataCache cache(path, 3);
    cache.open();

    cache.push("t", "old1");
    cache.push("t", "old2");
    cache.push("t", "old3");
    cache.push("t", "new1");  // 触发丢弃 old1

    // size 不应超过 3
    REQUIRE(cache.size() <= 3);

    auto batch = cache.pop_batch(10);
    // 最旧的条目（old1）应被丢弃，新条目在里面
    bool has_new = false, has_old1 = false;
    for (auto& r : batch) {
        if (r.payload == "new1")
            has_new = true;
        if (r.payload == "old1")
            has_old1 = true;
    }
    REQUIRE(has_new);
    REQUIRE_FALSE(has_old1);

    cache.close();
    fs::remove(path);
    fs::remove(path + ".tmp");
}

// ────────────────────────────────────────────────────────────────
// 断电持久化：open() 后再次 open() 恢复数据
// ────────────────────────────────────────────────────────────────
TEST_CASE("DataCache: 重新 open() 从文件恢复未确认记录", "[middleware][data_cache]") {
    std::string path = "/tmp/test_dc_persist.jsonl";
    {
        DataCache c1(path);
        c1.open();
        c1.push("topic", "payload_A");
        c1.push("topic", "payload_B");
        c1.close();
    }
    {
        DataCache c2(path);
        c2.open();
        REQUIRE(c2.size() == 2);
        auto batch = c2.pop_batch(10);
        REQUIRE(batch[0].payload == "payload_A");
        REQUIRE(batch[1].payload == "payload_B");
        c2.close();
    }
    fs::remove(path);
    fs::remove(path + ".tmp");
}

TEST_CASE("DataCache: ack journal survives restart without full rewrite semantics",
          "[middleware][data_cache]") {
    std::string path = "/tmp/test_dc_journal.jsonl";
    {
        DataCache c1(path);
        c1.open();
        c1.push("topic", "payload_A");
        c1.push("topic", "payload_B");
        auto batch = c1.pop_batch(10);
        REQUIRE(batch.size() == 2);
        c1.confirm_sent({batch[0].id});
        c1.close();
    }
    {
        DataCache c2(path);
        c2.open();
        auto batch = c2.pop_batch(10);
        REQUIRE(batch.size() == 1);
        REQUIRE(batch[0].payload == "payload_B");
        c2.close();
    }
    fs::remove(path);
    fs::remove(path + ".tmp");
}

TEST_CASE("DataCache: journal file keeps push and ack JSONL record format",
          "[middleware][data_cache]") {
    const std::string path = "/tmp/test_dc_journal_format.jsonl";
    fs::remove(path);
    fs::remove(path + ".tmp");

    DataCache cache(path);
    REQUIRE(cache.open());
    REQUIRE(cache.push("topic/test", R"({"v":1})", 1234));

    const auto batch = cache.pop_batch(10);
    REQUIRE(batch.size() == 1);
    cache.confirm_sent({batch[0].id});
    cache.close();

    std::ifstream in(path);
    REQUIRE(in.is_open());

    std::string push_line;
    std::string ack_line;
    REQUIRE(std::getline(in, push_line));
    REQUIRE(std::getline(in, ack_line));

    rapidjson::Document push_json;
    push_json.Parse(push_line.c_str());
    REQUIRE_FALSE(push_json.HasParseError());
    CHECK(std::string(push_json["op"].GetString()) == "push");
    CHECK(push_json["id"].GetInt64() == batch[0].id);
    CHECK(std::string(push_json["topic"].GetString()) == "topic/test");
    CHECK(std::string(push_json["payload"].GetString()) == R"({"v":1})");
    CHECK(push_json["ts_ms"].GetUint64() == 1234);

    rapidjson::Document ack_json;
    ack_json.Parse(ack_line.c_str());
    REQUIRE_FALSE(ack_json.HasParseError());
    CHECK(std::string(ack_json["op"].GetString()) == "ack");
    CHECK(ack_json["id"].GetInt64() == batch[0].id);

    fs::remove(path);
    fs::remove(path + ".tmp");
}

TEST_CASE("DataCache: compact snapshot keeps only live records",
          "[middleware][data_cache]") {
    std::string path = "/tmp/test_dc_compact.jsonl";
    {
        DataCache cache(path, 512);
        cache.open();
        for (int i = 0; i < 180; ++i) {
            REQUIRE(cache.push("topic", "payload_" + std::to_string(i), 1000 + i));
        }

        auto batch = cache.pop_batch(180);
        REQUIRE(batch.size() == 180);

        std::vector<int64_t> ack_ids;
        for (int i = 0; i < 150; ++i) {
            ack_ids.push_back(batch[static_cast<size_t>(i)].id);
        }
        cache.confirm_sent(ack_ids);
        REQUIRE(cache.size() == 30);
        cache.close();
    }

    {
        std::ifstream in(path);
        REQUIRE(in.is_open());
        std::string line;
        size_t line_count = 0;
        while (std::getline(in, line)) {
            if (!line.empty()) ++line_count;
            REQUIRE(line.find("\"op\"") == std::string::npos);
        }
        REQUIRE(line_count == 30);
    }

    {
        DataCache cache(path, 512);
        REQUIRE(cache.open());
        REQUIRE(cache.size() == 30);
        auto batch = cache.pop_batch(40);
        REQUIRE(batch.size() == 30);
        REQUIRE(batch.front().payload == "payload_150");
        REQUIRE(batch.back().payload == "payload_179");
    }

    fs::remove(path);
    fs::remove(path + ".tmp");
}

TEST_CASE("DataCache: full-queue eviction does not drop a record when ack journal append fails",
          "[middleware][data_cache]") {
    const std::string path = "/tmp/test_dc_eviction_ack_failure.jsonl";
    fs::remove(path);
    fs::remove(path + ".tmp");

    DataCache cache(path, 2);
    REQUIRE(cache.open());
    REQUIRE(cache.push("t/a", R"({"v":1})"));
    REQUIRE(cache.push("t/b", R"({"v":2})"));

    cache.set_test_append_hook([](const rapidjson::Document& j) {
        const auto op_it = j.FindMember("op");
        return op_it == j.MemberEnd() || std::string(op_it->value.GetString()) != "ack";
    });

    REQUIRE_FALSE(cache.push("t/c", R"({"v":3})"));
    CHECK(cache.size() == 2);

    const auto batch = cache.pop_batch(10);
    REQUIRE(batch.size() == 2);
    CHECK(batch[0].payload == R"({"v":1})");
    CHECK(batch[1].payload == R"({"v":2})");

    cache.close();
    fs::remove(path);
    fs::remove(path + ".tmp");
}
