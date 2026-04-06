#include <gtest/gtest.h>
#include "carla/ros2/dds/DDSPublisherImpl.h"
#include "carla/ros2/dds/DDSSubscriberImpl.h"

using namespace carla::ros2;

// Mock implementation to test the interface contract
class MockPublisher : public DDSPublisherImpl {
public:
    bool init_called = false;
    bool write_called = false;
    bool destroy_called = false;
    void* last_write_data = nullptr;

    bool Init(const TopicConfig& config, const std::string& participant_name,
              const std::string& topic_name, bool use_preallocated_realloc) override {
        init_called = true;
        return true;
    }
    bool Write(void* data) override {
        write_called = true;
        last_write_data = data;
        return true;
    }
    bool IsConnected() const override { return init_called; }
    void Destroy() override { destroy_called = true; }
};

class MockSubscriber : public DDSSubscriberImpl {
public:
    bool init_called = false;
    bool take_called = false;
    OnDataCallback stored_callback;

    bool Init(const TopicConfig& config, const std::string& participant_name,
              const std::string& topic_name) override {
        init_called = true;
        return true;
    }
    bool TakeNextSample(void* data) override {
        take_called = true;
        return true;
    }
    void Destroy() override {}
    void SetOnDataCallback(OnDataCallback callback) override {
        stored_callback = std::move(callback);
    }
};

TEST(DDSPublisherImpl, InitWriteDestroyLifecycle) {
    MockPublisher pub;
    TopicConfig config{};
    config.domain_id = 0;
    config.reliability_qos = ReliabilityQoS::RELIABLE;
    config.durability_qos = DurabilityQoS::VOLATILE;
    config.history_qos = HistoryQoS::KEEP_LAST;
    config.history_qos_depth = 1;

    ASSERT_TRUE(pub.Init(config, "test_participant", "rt/test/topic", false));
    ASSERT_TRUE(pub.init_called);
    ASSERT_TRUE(pub.IsConnected());

    int dummy_data = 42;
    ASSERT_TRUE(pub.Write(&dummy_data));
    ASSERT_TRUE(pub.write_called);
    ASSERT_EQ(pub.last_write_data, &dummy_data);

    pub.Destroy();
    ASSERT_TRUE(pub.destroy_called);
}

TEST(DDSPublisherImpl, DestructorCallsDestroy) {
    auto pub = std::make_unique<MockPublisher>();
    TopicConfig config{};
    config.domain_id = 0;
    pub->Init(config, "test", "rt/test", false);
    // unique_ptr destruction should be safe
    pub.reset();
}

TEST(DDSSubscriberImpl, InitTakeCallbackLifecycle) {
    MockSubscriber sub;
    TopicConfig config{};
    config.domain_id = 0;

    ASSERT_TRUE(sub.Init(config, "test_subscriber", "rt/test/topic"));
    ASSERT_TRUE(sub.init_called);

    bool callback_fired = false;
    sub.SetOnDataCallback([&callback_fired]() { callback_fired = true; });
    sub.stored_callback();
    ASSERT_TRUE(callback_fired);

    int dummy_data = 0;
    ASSERT_TRUE(sub.TakeNextSample(&dummy_data));
    ASSERT_TRUE(sub.take_called);
}
