/**
 * @file lockfree_queue_test.cc
 * @brief boost::lockfree::spsc_queue 无锁单生产者单消费者队列单元测试
 *
 * 测试分组：[driver][nonlock]
 *
 * 运行方法：
 *   ./unit_tests "[driver][nonlock]"
 */
#include <boost/lockfree/spsc_queue.hpp>
#include <cassert>
#include <catch2/catch.hpp>

TEST_CASE("无锁队列", "[driver][nonlock]") {
    boost::lockfree::spsc_queue<int, boost::lockfree::capacity<1024>> q;

    assert(q.empty());

    for (int i = 0; i < 100; ++i) {
        assert(q.push(i));
    }

    int v;
    for (int i = 0; i < 100; ++i) {
        assert(q.pop(v));
        assert(v == i);
    }

    assert(q.empty());
}
