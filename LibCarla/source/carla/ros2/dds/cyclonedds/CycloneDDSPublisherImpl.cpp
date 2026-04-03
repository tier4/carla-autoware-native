// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CycloneDDSPublisherImpl.h"
#include "CycloneDDSConversions.hpp"

#include <iostream>

namespace carla {
namespace ros2 {

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------
CycloneDDSPublisherImpl::CycloneDDSPublisherImpl(const dds_topic_descriptor_t* descriptor)
    : _descriptor(descriptor)
{}

CycloneDDSPublisherImpl::~CycloneDDSPublisherImpl() {
    Destroy();
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
bool CycloneDDSPublisherImpl::Init(
        const TopicConfig& config,
        const std::string& participant_name,
        const std::string& topic_name,
        bool /*use_preallocated_realloc*/)
{
    if (_descriptor == nullptr) {
        std::cerr << "CycloneDDSPublisherImpl::Init: null topic descriptor "
                  << "(type not yet available)" << std::endl;
        return false;
    }

    // --- DomainParticipant ---
    _participant = dds_create_participant(
        static_cast<dds_domainid_t>(config.domain_id), nullptr, nullptr);
    if (_participant < 0) {
        std::cerr << "CycloneDDSPublisherImpl: failed to create participant ("
                  << dds_strretcode(-_participant) << ")" << std::endl;
        return false;
    }

    // --- Topic ---
    _topic = dds_create_topic(
        _participant, _descriptor, topic_name.c_str(), nullptr, nullptr);
    if (_topic < 0) {
        std::cerr << "CycloneDDSPublisherImpl: failed to create topic '"
                  << topic_name << "' (" << dds_strretcode(-_topic) << ")"
                  << std::endl;
        Destroy();
        return false;
    }

    // --- QoS ---
    dds_qos_t* qos = dds_create_qos();
    configure_cyclone_qos(config, qos);

    // --- DataWriter ---
    _writer = dds_create_writer(_participant, _topic, qos, nullptr);
    dds_delete_qos(qos);

    if (_writer < 0) {
        std::cerr << "CycloneDDSPublisherImpl: failed to create writer ("
                  << dds_strretcode(-_writer) << ")" << std::endl;
        Destroy();
        return false;
    }

    _initialized = true;
    return true;
}

// ---------------------------------------------------------------------------
// Write
// ---------------------------------------------------------------------------
bool CycloneDDSPublisherImpl::Write(void* data) {
    if (!_initialized) {
        std::cerr << "CycloneDDSPublisherImpl::Write: not initialized" << std::endl;
        return false;
    }

    dds_return_t rc = dds_write(_writer, data);
    if (rc != DDS_RETCODE_OK) {
        std::cerr << "CycloneDDSPublisherImpl::Write: dds_write failed ("
                  << dds_strretcode(-rc) << ")" << std::endl;
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// IsConnected
// ---------------------------------------------------------------------------
bool CycloneDDSPublisherImpl::IsConnected() const {
    if (!_initialized) return false;

    // Check publication matched status to determine if any subscribers are connected
    dds_publication_matched_status_t status;
    dds_return_t rc = dds_get_publication_matched_status(_writer, &status);
    if (rc != DDS_RETCODE_OK) return false;

    return status.current_count > 0;
}

// ---------------------------------------------------------------------------
// Destroy
// ---------------------------------------------------------------------------
void CycloneDDSPublisherImpl::Destroy() {
    if (_participant > 0) {
        // dds_delete on participant recursively deletes all children
        // (topic, writer, etc.)
        dds_delete(_participant);
    }
    _participant = 0;
    _writer = 0;
    _topic = 0;
    _initialized = false;
}

} // namespace ros2
} // namespace carla
