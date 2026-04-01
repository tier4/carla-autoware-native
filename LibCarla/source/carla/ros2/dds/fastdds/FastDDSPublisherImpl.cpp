// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "FastDDSPublisherImpl.h"
#include "FastDDSConversions.hpp"

#include <iostream>

#include <fastdds/dds/core/status/PublicationMatchedStatus.hpp>
#include <fastrtps/rtps/common/InstanceHandle.h>

namespace carla {
namespace ros2 {

// ---------------------------------------------------------------------------
// Nested DataWriterListener implementation
// ---------------------------------------------------------------------------
class FastDDSPublisherImpl::ListenerImpl : public efd::DataWriterListener {
public:
    void on_publication_matched(
            efd::DataWriter* writer,
            const efd::PublicationMatchedStatus& info) override
    {
        if (info.current_count_change == 1) {
            _matched = info.total_count;
            _first_connected = true;
        } else if (info.current_count_change == -1) {
            _matched = info.total_count;
        } else {
            std::cerr << info.current_count_change
                      << " is not a valid value for PublicationMatchedStatus current count change"
                      << std::endl;
        }
    }

    int _matched { 0 };
    bool _first_connected { false };
};

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------
FastDDSPublisherImpl::FastDDSPublisherImpl(efd::TypeSupport type_support)
    : _type(std::move(type_support))
    , _listener(std::make_unique<ListenerImpl>())
{}

FastDDSPublisherImpl::~FastDDSPublisherImpl() {
    Destroy();
}

// ---------------------------------------------------------------------------
// Init  (mirrors the existing per-publisher Init pattern)
// ---------------------------------------------------------------------------
bool FastDDSPublisherImpl::Init(
        const TopicConfig& config,
        const std::string& participant_name,
        const std::string& topic_name,
        bool use_preallocated_realloc)
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

    // --- Publisher ---
    efd::PublisherQos pubqos = efd::PUBLISHER_QOS_DEFAULT;
    _publisher = _participant->create_publisher(pubqos, nullptr);
    if (_publisher == nullptr) {
        std::cerr << "Failed to create Publisher" << std::endl;
        return false;
    }

    // --- Topic ---
    efd::TopicQos tqos = efd::TOPIC_QOS_DEFAULT;
    _topic = _participant->create_topic(topic_name, _type->getName(), tqos);
    if (_topic == nullptr) {
        std::cerr << "Failed to create Topic" << std::endl;
        return false;
    }

    // --- DataWriter ---
    efd::DataWriterQos wqos = efd::DATAWRITER_QOS_DEFAULT;
    configure_qos(config, wqos);

    if (use_preallocated_realloc) {
        wqos.endpoint().history_memory_policy =
            eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
    }

    efd::DataWriterListener* listener =
        static_cast<efd::DataWriterListener*>(_listener.get());
    _datawriter = _publisher->create_datawriter(_topic, wqos, listener);
    if (_datawriter == nullptr) {
        std::cerr << "Failed to create DataWriter" << std::endl;
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Write
// ---------------------------------------------------------------------------
bool FastDDSPublisherImpl::Write(void* data) {
    eprosima::fastrtps::rtps::InstanceHandle_t instance_handle;
    eprosima::fastrtps::types::ReturnCode_t rcode =
        _datawriter->write(data, instance_handle);

    if (rcode == erc::ReturnCodeValue::RETCODE_OK) {
        return true;
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
    if (rcode == erc::ReturnCodeValue::RETCODE_NO_DATA) {
        std::cerr << "RETCODE_NO_DATA" << std::endl;
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
// IsConnected
// ---------------------------------------------------------------------------
bool FastDDSPublisherImpl::IsConnected() const {
    return _listener && _listener->_first_connected;
}

// ---------------------------------------------------------------------------
// Destroy  (mirrors the existing per-publisher destructor pattern)
// ---------------------------------------------------------------------------
void FastDDSPublisherImpl::Destroy() {
    if (_datawriter) {
        _publisher->delete_datawriter(_datawriter);
        _datawriter = nullptr;
    }
    if (_publisher) {
        _participant->delete_publisher(_publisher);
        _publisher = nullptr;
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
