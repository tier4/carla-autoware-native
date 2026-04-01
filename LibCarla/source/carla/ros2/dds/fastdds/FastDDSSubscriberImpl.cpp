// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "FastDDSSubscriberImpl.h"
#include "FastDDSConversions.hpp"

#include <iostream>

namespace carla {
namespace ros2 {

// ---------------------------------------------------------------------------
// Nested DataReaderListener implementation
// ---------------------------------------------------------------------------
class FastDDSSubscriberImpl::ListenerImpl : public efd::DataReaderListener {
public:
    void on_data_available(efd::DataReader* reader) override {
        if (_callback) _callback();
    }

    void on_subscription_matched(
            efd::DataReader* reader,
            const efd::SubscriptionMatchedStatus& info) override
    {
        if (info.current_count_change == 1) {
            _matched = info.total_count;
        } else if (info.current_count_change == -1) {
            _matched = info.total_count;
        } else {
            std::cerr << info.current_count_change
                      << " is not a valid value for SubscriptionMatchedStatus current count change"
                      << std::endl;
        }
    }

    OnDataCallback _callback;
    int _matched { 0 };
};

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------
FastDDSSubscriberImpl::FastDDSSubscriberImpl(efd::TypeSupport type_support)
    : _type(std::move(type_support))
    , _listener(std::make_unique<ListenerImpl>())
{}

FastDDSSubscriberImpl::~FastDDSSubscriberImpl() {
    Destroy();
}

// ---------------------------------------------------------------------------
// Init  (mirrors the existing per-subscriber Init pattern)
// ---------------------------------------------------------------------------
bool FastDDSSubscriberImpl::Init(
        const TopicConfig& config,
        const std::string& participant_name,
        const std::string& topic_name)
{
    if (_type == nullptr) {
        std::cerr << "Invalid TypeSupport" << std::endl;
        return false;
    }

    // --- DomainParticipant ---
    efd::DomainParticipantQos pqos = efd::PARTICIPANT_QOS_DEFAULT;
    pqos.name(participant_name);
    auto factory = efd::DomainParticipantFactory::get_instance();
    _participant = factory->create_participant(config.domain_id, pqos);
    if (_participant == nullptr) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }
    _type.register_type(_participant);

    // --- Subscriber ---
    efd::SubscriberQos subqos = efd::SUBSCRIBER_QOS_DEFAULT;
    _subscriber = _participant->create_subscriber(subqos, nullptr);
    if (_subscriber == nullptr) {
        std::cerr << "Failed to create Subscriber" << std::endl;
        return false;
    }

    // --- Topic ---
    efd::TopicQos tqos = efd::TOPIC_QOS_DEFAULT;
    _topic = _participant->create_topic(topic_name, _type->getName(), tqos);
    if (_topic == nullptr) {
        std::cerr << "Failed to create Topic" << std::endl;
        return false;
    }

    // --- DataReader ---
    efd::DataReaderQos rqos = efd::DATAREADER_QOS_DEFAULT;
    configure_qos(config, rqos);

    efd::DataReaderListener* listener =
        static_cast<efd::DataReaderListener*>(_listener.get());
    _datareader = _subscriber->create_datareader(_topic, rqos, listener);
    if (_datareader == nullptr) {
        std::cerr << "Failed to create DataReader" << std::endl;
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// TakeNextSample
// ---------------------------------------------------------------------------
bool FastDDSSubscriberImpl::TakeNextSample(void* data) {
    efd::SampleInfo info;
    eprosima::fastrtps::types::ReturnCode_t rcode =
        _datareader->take_next_sample(data, &info);

    if (rcode == erc::ReturnCodeValue::RETCODE_OK) {
        return true;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_NO_DATA) {
        return false;  // Normal: no data available yet
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_ERROR) {
        std::cerr << "RETCODE_ERROR" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_UNSUPPORTED) {
        std::cerr << "RETCODE_UNSUPPORTED" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_BAD_PARAMETER) {
        std::cerr << "RETCODE_BAD_PARAMETER" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_PRECONDITION_NOT_MET) {
        std::cerr << "RETCODE_PRECONDITION_NOT_MET" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_OUT_OF_RESOURCES) {
        std::cerr << "RETCODE_OUT_OF_RESOURCES" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_NOT_ENABLED) {
        std::cerr << "RETCODE_NOT_ENABLED" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_IMMUTABLE_POLICY) {
        std::cerr << "RETCODE_IMMUTABLE_POLICY" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_INCONSISTENT_POLICY) {
        std::cerr << "RETCODE_INCONSISTENT_POLICY" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_ALREADY_DELETED) {
        std::cerr << "RETCODE_ALREADY_DELETED" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_TIMEOUT) {
        std::cerr << "RETCODE_TIMEOUT" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_ILLEGAL_OPERATION) {
        std::cerr << "RETCODE_ILLEGAL_OPERATION" << std::endl;
        return false;
    }
    if (rcode == erc::ReturnCodeValue::RETCODE_NOT_ALLOWED_BY_SECURITY) {
        std::cerr << "RETCODE_NOT_ALLOWED_BY_SECURITY" << std::endl;
        return false;
    }
    std::cerr << "UNKNOWN" << std::endl;
    return false;
}

// ---------------------------------------------------------------------------
// SetOnDataCallback
// ---------------------------------------------------------------------------
void FastDDSSubscriberImpl::SetOnDataCallback(OnDataCallback callback) {
    _listener->_callback = std::move(callback);
}

// ---------------------------------------------------------------------------
// Destroy  (mirrors the existing per-subscriber destructor pattern)
// ---------------------------------------------------------------------------
void FastDDSSubscriberImpl::Destroy() {
    if (_datareader) {
        _subscriber->delete_datareader(_datareader);
        _datareader = nullptr;
    }
    if (_subscriber) {
        _participant->delete_subscriber(_subscriber);
        _subscriber = nullptr;
    }
    if (_topic) {
        _participant->delete_topic(_topic);
        _topic = nullptr;
    }
    if (_participant) {
        efd::DomainParticipantFactory::get_instance()->delete_participant(_participant);
        _participant = nullptr;
    }
}

} // namespace ros2
} // namespace carla
