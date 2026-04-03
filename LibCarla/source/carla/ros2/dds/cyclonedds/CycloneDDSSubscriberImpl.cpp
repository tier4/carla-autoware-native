// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CycloneDDSSubscriberImpl.h"
#include "CycloneDDSConversions.hpp"

#include <iostream>

namespace carla {
namespace ros2 {

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------
CycloneDDSSubscriberImpl::CycloneDDSSubscriberImpl(const dds_topic_descriptor_t* descriptor)
    : _descriptor(descriptor)
{}

CycloneDDSSubscriberImpl::~CycloneDDSSubscriberImpl() {
    Destroy();
}

// ---------------------------------------------------------------------------
// Listener callback (static)
// ---------------------------------------------------------------------------
void CycloneDDSSubscriberImpl::on_data_available(dds_entity_t /*reader*/, void* arg) {
    auto* self = static_cast<CycloneDDSSubscriberImpl*>(arg);
    if (self && self->_callback) {
        self->_callback();
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
bool CycloneDDSSubscriberImpl::Init(
        const TopicConfig& config,
        const std::string& participant_name,
        const std::string& topic_name)
{
    if (_descriptor == nullptr) {
        std::cerr << "CycloneDDSSubscriberImpl::Init: null topic descriptor "
                  << "(type not yet available)" << std::endl;
        return false;
    }

    // --- DomainParticipant ---
    _participant = dds_create_participant(
        static_cast<dds_domainid_t>(config.domain_id), nullptr, nullptr);
    if (_participant < 0) {
        std::cerr << "CycloneDDSSubscriberImpl: failed to create participant ("
                  << dds_strretcode(-_participant) << ")" << std::endl;
        return false;
    }

    // --- Topic ---
    _topic = dds_create_topic(
        _participant, _descriptor, topic_name.c_str(), nullptr, nullptr);
    if (_topic < 0) {
        std::cerr << "CycloneDDSSubscriberImpl: failed to create topic '"
                  << topic_name << "' (" << dds_strretcode(-_topic) << ")"
                  << std::endl;
        Destroy();
        return false;
    }

    // --- QoS ---
    dds_qos_t* qos = dds_create_qos();
    configure_cyclone_qos(config, qos);

    // --- Listener (optional, fires callback on data available) ---
    dds_listener_t* listener = nullptr;
    if (_callback) {
        listener = dds_create_listener(this);
        dds_lset_data_available(listener, on_data_available);
    }

    // --- DataReader ---
    _reader = dds_create_reader(_participant, _topic, qos, listener);
    dds_delete_qos(qos);
    if (listener) {
        dds_delete_listener(listener);
    }

    if (_reader < 0) {
        std::cerr << "CycloneDDSSubscriberImpl: failed to create reader ("
                  << dds_strretcode(-_reader) << ")" << std::endl;
        Destroy();
        return false;
    }

    _initialized = true;
    return true;
}

// ---------------------------------------------------------------------------
// TakeNextSample
// ---------------------------------------------------------------------------
bool CycloneDDSSubscriberImpl::TakeNextSample(void* data) {
    if (!_initialized) {
        std::cerr << "CycloneDDSSubscriberImpl::TakeNextSample: not initialized"
                  << std::endl;
        return false;
    }

    void* samples[1] = { data };
    dds_sample_info_t infos[1];

    int32_t n = dds_take(_reader, samples, infos, 1, 1);
    if (n < 0) {
        std::cerr << "CycloneDDSSubscriberImpl::TakeNextSample: dds_take failed ("
                  << dds_strretcode(-n) << ")" << std::endl;
        return false;
    }
    if (n == 0) {
        return false;  // No data available yet
    }

    // Only return true if sample contains valid data
    return infos[0].valid_data;
}

// ---------------------------------------------------------------------------
// SetOnDataCallback
// ---------------------------------------------------------------------------
void CycloneDDSSubscriberImpl::SetOnDataCallback(OnDataCallback callback) {
    _callback = std::move(callback);

    // If the reader is already created, update the listener on the fly
    if (_initialized && _reader > 0) {
        if (_callback) {
            dds_listener_t* listener = dds_create_listener(this);
            dds_lset_data_available(listener, on_data_available);
            dds_set_listener(_reader, listener);
            dds_delete_listener(listener);
        } else {
            dds_set_listener(_reader, nullptr);
        }
    }
}

// ---------------------------------------------------------------------------
// Destroy
// ---------------------------------------------------------------------------
void CycloneDDSSubscriberImpl::Destroy() {
    if (_participant > 0) {
        // dds_delete on participant recursively deletes all children
        // (topic, reader, etc.)
        dds_delete(_participant);
    }
    _participant = 0;
    _reader = 0;
    _topic = 0;
    _initialized = false;
}

} // namespace ros2
} // namespace carla
