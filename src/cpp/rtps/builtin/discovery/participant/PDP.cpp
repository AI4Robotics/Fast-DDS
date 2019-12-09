// Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file PDP.cpp
 *
 */

#include <fastrtps/rtps/builtin/discovery/participant/PDP.h>
#include <fastrtps/rtps/builtin/discovery/participant/PDPListener.h>

#include <fastrtps/rtps/builtin/BuiltinProtocols.h>
#include <fastrtps/rtps/builtin/liveliness/WLP.h>

#include <fastrtps/rtps/builtin/data/ParticipantProxyData.h>
#include <fastrtps/rtps/participant/RTPSParticipantListener.h>
#include <fastrtps/rtps/resources/TimedEvent.h>
#include <fastrtps/rtps/builtin/data/ReaderProxyData.h>
#include <fastrtps/rtps/builtin/data/WriterProxyData.h>

#include <fastrtps/rtps/builtin/discovery/endpoint/EDPSimple.h>
#include <fastrtps/rtps/builtin/discovery/endpoint/EDPStatic.h>

#include <fastrtps/rtps/resources/AsyncWriterThread.h>

#include "../../../participant/RTPSParticipantImpl.h"

#include <fastrtps/rtps/writer/StatelessWriter.h>
#include <fastrtps/rtps/reader/StatelessReader.h>
#include <fastrtps/rtps/reader/StatefulReader.h>

#include <fastrtps/rtps/history/WriterHistory.h>
#include <fastrtps/rtps/history/ReaderHistory.h>


#include <fastrtps/utils/TimeConversion.h>
#include <fastrtps/utils/IPLocator.h>

#include <fastrtps/log/Log.h>

#include <mutex>
#include <chrono>

namespace eprosima {
namespace fastrtps {
namespace rtps {

// Default configuration values for PDP reliable entities.

const Duration_t pdp_heartbeat_period{ 0, 350 * 1000  }; // 350 milliseconds
const Duration_t pdp_nack_response_delay{ 0, 100 * 1000  }; // 100 milliseconds
const Duration_t pdp_nack_supression_duration{ 0, 11 * 1000 }; // ~11 milliseconds
const Duration_t pdp_heartbeat_response_delay{ 0, 11 * 1000 }; // ~11 milliseconds

const int32_t pdp_initial_reserved_caches = 20;

// Static pool resources shared among all participants
std::recursive_mutex PDP::pool_mutex_;
size_t PDP::pdp_counter_ = 0;

size_t PDP::participant_proxies_number_ = 0;
std::vector<ParticipantProxyData*> PDP::participant_proxies_pool_;

size_t PDP::reader_proxies_number_ = 0;
std::vector<ReaderProxyData*> PDP::reader_proxies_pool_;

size_t PDP::writer_proxies_number_ = 0;
std::vector<WriterProxyData*> PDP::writer_proxies_pool_;

std::map<GuidPrefix_t, std::weak_ptr<ParticipantProxyData>> PDP::pool_participant_references_;

ResourceEvent PDP::event_thr_;

PDP::PDP (
        BuiltinProtocols* built,
        const RTPSParticipantAllocationAttributes& allocation)
    : mp_builtin(built)
    , mp_RTPSParticipant(nullptr)
    , mp_PDPWriter(nullptr)
    , mp_PDPReader(nullptr)
    , mp_EDP(nullptr)
    , m_hasChangedLocalPDP(true)
    , mp_listener(nullptr)
    , mp_PDPWriterHistory(nullptr)
    , mp_PDPReaderHistory(nullptr)
    , temp_reader_data_(allocation.locators.max_unicast_locators, allocation.locators.max_multicast_locators)
    , temp_writer_data_(allocation.locators.max_unicast_locators, allocation.locators.max_multicast_locators)
    , mp_mutex(new std::recursive_mutex())
    , participant_proxies_(allocation.participants)
    , resend_participant_info_event_(nullptr)
{
    initialize_or_update_pool_allocation(allocation);
}

PDP::~PDP()
{
    delete resend_participant_info_event_;
    mp_RTPSParticipant->disableReader(mp_PDPReader);
    delete mp_EDP;
    mp_RTPSParticipant->deleteUserEndpoint(mp_PDPWriter);
    mp_RTPSParticipant->deleteUserEndpoint(mp_PDPReader);
    delete mp_PDPWriterHistory;
    delete mp_PDPReaderHistory;
    delete mp_listener;

    participant_proxies_.clear();

    delete mp_mutex;

    remove_pool_resources();
}

void  PDP::add_participant_proxy_data(
    std::shared_ptr<ParticipantProxyData> & ppd)
{
    std::lock_guard<std::recursive_mutex> pdp_lock(*mp_mutex);
    participant_proxies_.push_back(ppd);
}

std::shared_ptr<ParticipantProxyData> PDP::add_participant_proxy_data(
        const GUID_t& participant_guid,
        bool with_lease_duration)
{
    std::shared_ptr<ParticipantProxyData> ret_val;

    {
        std::unique_lock<std::recursive_mutex> pool_guard(PDP::pool_mutex_);

        //See whether it is already in use
        auto participant_reference = pool_participant_references_.find(participant_guid.guidPrefix);
        if(participant_reference != pool_participant_references_.end())
        {
            ret_val = participant_reference->second.lock();

            // locks on the ParticipantProxyData, this method exits with this lock taken
            ret_val->ppd_mutex_.lock();

            // It should be an associated ParticipantProxyData
            assert(!!ret_val);
        }
        else
        {
            // Try to take one entry from the pool
            if(participant_proxies_pool_.empty())
            {
                size_t max_proxies = participant_proxies_.max_size();
                if(participant_proxies_number_ < max_proxies)
                {
                    // Pool is empty but limit has not been reached, so we create a new entry.
                    ret_val = std::shared_ptr<ParticipantProxyData>(
                        new ParticipantProxyData(mp_RTPSParticipant->getRTPSParticipantAttributes().allocation),
                        ParticipantProxyData::pool_deleter());

                    if(!ret_val)
                    {
                        return ret_val;
                    }

                    ++participant_proxies_number_;

                    if(participant_guid != mp_RTPSParticipant->getGuid())
                    {
                        pool_guard.unlock();

                        {
                            GuidPrefix_t own_prefix = getLocalParticipantProxyData()->m_guid.guidPrefix; 
                            ret_val->lease_callback_.add_listener(own_prefix, this);
                        }
                        pool_guard.lock();
                       
                    }
                }
                else
                {
                    logWarning(RTPS_PDP, "Maximum number of participant proxies (" << max_proxies << \
                        ") reached for participant " << mp_RTPSParticipant->getGuid() << std::endl);
                    return nullptr;
                }
            }
            else
            {
                // Pool is not empty, use entry from pool
                ret_val.reset(participant_proxies_pool_.back(), ParticipantProxyData::pool_deleter());
                participant_proxies_pool_.pop_back();
            }

            // locks on the ParticipantProxyData, this method exits with this lock taken
            ret_val->ppd_mutex_.lock();

            ret_val->should_check_lease_duration = with_lease_duration;
            ret_val->m_guid = participant_guid;
        }

        // Add returned entry to the collection
        pool_participant_references_[participant_guid.guidPrefix] = std::weak_ptr<ParticipantProxyData>(ret_val);
    }
 
    add_participant_proxy_data(ret_val);


    return ret_val;
}

void PDP::initializeParticipantProxyData(ParticipantProxyData* participant_data)
{
    std::lock_guard<std::recursive_mutex> ppd_lock(participant_data->ppd_mutex_);


    // Signal out is the first announcement to avoid deserialization from all other intra process participants
    participant_data->version_ = SequenceNumber_t(0, 1);

    participant_data->m_leaseDuration = mp_RTPSParticipant->getAttributes().builtin.discovery_config.leaseDuration;
    //set_VendorId_eProsima(participant_data->m_VendorId);
    participant_data->m_VendorId = c_VendorId_eProsima;

    participant_data->m_availableBuiltinEndpoints |= DISC_BUILTIN_ENDPOINT_PARTICIPANT_ANNOUNCER;
    participant_data->m_availableBuiltinEndpoints |= DISC_BUILTIN_ENDPOINT_PARTICIPANT_DETECTOR;

#if HAVE_SECURITY
    participant_data->m_availableBuiltinEndpoints |= DISC_BUILTIN_ENDPOINT_PARTICIPANT_SECURE_ANNOUNCER;
    participant_data->m_availableBuiltinEndpoints |= DISC_BUILTIN_ENDPOINT_PARTICIPANT_SECURE_DETECTOR;
#endif

    if(mp_RTPSParticipant->getAttributes().builtin.use_WriterLivelinessProtocol)
    {
        participant_data->m_availableBuiltinEndpoints |= BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_DATA_WRITER;
        participant_data->m_availableBuiltinEndpoints |= BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_DATA_READER;

#if HAVE_SECURITY
        participant_data->m_availableBuiltinEndpoints |= BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_SECURE_DATA_WRITER;
        participant_data->m_availableBuiltinEndpoints |= BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_SECURE_DATA_READER;
#endif
    }

#if HAVE_SECURITY
    participant_data->m_availableBuiltinEndpoints |= mp_RTPSParticipant->security_manager().builtin_endpoints();
#endif

    for (const Locator_t& loc : mp_RTPSParticipant->getAttributes().defaultUnicastLocatorList)
    {
        participant_data->default_locators.add_unicast_locator(loc);
    }
    for (const Locator_t& loc : mp_RTPSParticipant->getAttributes().defaultMulticastLocatorList)
    {
        participant_data->default_locators.add_multicast_locator(loc);
    }
    participant_data->m_expectsInlineQos = false;
    participant_data->m_guid = mp_RTPSParticipant->getGuid();
    for(uint8_t i = 0; i<16; ++i)
    {
        if(i<12)
            participant_data->m_key.value[i] = participant_data->m_guid.guidPrefix.value[i];
        else
            participant_data->m_key.value[i] = participant_data->m_guid.entityId.value[i - 12];
    }

    // Keep persistence Guid_Prefix_t in a specific property. This info must be propagated to all builtin endpoints
    {
        GuidPrefix_t persistent = mp_RTPSParticipant->getAttributes().prefix;

        if(persistent != c_GuidPrefix_Unknown)
        {
            participant_data->set_persistence_guid(
                GUID_t(
                    persistent,
                    c_EntityId_RTPSParticipant));
        }
    }

    participant_data->metatraffic_locators.unicast.clear();
    for (const Locator_t& loc : this->mp_builtin->m_metatrafficUnicastLocatorList)
    {
        participant_data->metatraffic_locators.add_unicast_locator(loc);
    }

    participant_data->metatraffic_locators.multicast.clear();
    if (!m_discovery.avoid_builtin_multicast || participant_data->metatraffic_locators.unicast.empty())
    {
        for(const Locator_t& loc: this->mp_builtin->m_metatrafficMulticastLocatorList)
        {
            participant_data->metatraffic_locators.add_multicast_locator(loc);
        }
    }

    participant_data->m_participantName = std::string(mp_RTPSParticipant->getAttributes().getName());

    participant_data->m_userData = mp_RTPSParticipant->getAttributes().userData;

#if HAVE_SECURITY
    IdentityToken* identity_token = nullptr;
    if(mp_RTPSParticipant->security_manager().get_identity_token(&identity_token) && identity_token != nullptr)
    {
        participant_data->identity_token_ = std::move(*identity_token);
        mp_RTPSParticipant->security_manager().return_identity_token(identity_token);
    }

    PermissionsToken* permissions_token = nullptr;
    if(mp_RTPSParticipant->security_manager().get_permissions_token(&permissions_token)
        && permissions_token != nullptr)
    {
        participant_data->permissions_token_ = std::move(*permissions_token);
        mp_RTPSParticipant->security_manager().return_permissions_token(permissions_token);
    }

    if (mp_RTPSParticipant->is_secure())
    {
        const security::ParticipantSecurityAttributes & sec_attrs = mp_RTPSParticipant->security_attributes();
        participant_data->security_attributes_ = sec_attrs.mask();
        participant_data->plugin_security_attributes_ = sec_attrs.plugin_participant_attributes;
    }
    else
    {
        participant_data->security_attributes_ = 0UL;
        participant_data->plugin_security_attributes_ = 0UL;
    }
#endif
}

bool PDP::initPDP(
    RTPSParticipantImpl* part)
{
    logInfo(RTPS_PDP,"Beginning");
    mp_RTPSParticipant = part;
    m_discovery = mp_RTPSParticipant->getAttributes().builtin;
    initial_announcements_ = m_discovery.discovery_config.initial_announcements;
    //CREATE ENDPOINTS
    if (!createPDPEndpoints())
    {
        return false;
    }
    //UPDATE METATRAFFIC.
    mp_builtin->updateMetatrafficLocators(this->mp_PDPReader->getAttributes().unicastLocatorList);
    std::shared_ptr<ParticipantProxyData> pdata = add_participant_proxy_data(part->getGuid(), true);

    if (!pdata)
    {
        return false;
    }

    pdata->ppd_mutex_.unlock(); // add_participant_proxy_data locks on ParticipantProxyData mutex
    // nobody knows about him thus we can unlock

    initializeParticipantProxyData(pdata.get());

    resend_participant_info_event_ = new TimedEvent(mp_RTPSParticipant->getEventResource(),
            [&]() -> bool
            {
                announceParticipantState(false);
                set_next_announcement_interval();
                return true;
            },
            0);

    set_initial_announcement_interval();

    return true;
}

bool PDP::enable()
{
    return mp_RTPSParticipant->enableReader(mp_PDPReader);
}

void PDP::announceParticipantState(
    bool new_change,
    bool dispose,
    WriteParams& wparams)
{
    logInfo(RTPS_PDP,"Announcing RTPSParticipant State (new change: "<< new_change <<")");
    CacheChange_t* change = nullptr;

    if(!dispose)
    {
        if(m_hasChangedLocalPDP.exchange(false) || new_change)
        {
            this->mp_mutex->lock();
            std::shared_ptr<ParticipantProxyData> local_participant_data = getLocalParticipantProxyData();
            InstanceHandle_t key = local_participant_data->m_key;
            ParticipantProxyData proxy_data_copy(*local_participant_data);
            this->mp_mutex->unlock();

            if(mp_PDPWriterHistory->getHistorySize() > 0)
                mp_PDPWriterHistory->remove_min_change();
            // TODO(Ricardo) Change DISCOVERY_PARTICIPANT_DATA_MAX_SIZE with getLocalParticipantProxyData()->size().
            change = mp_PDPWriter->new_change([]() -> uint32_t
                {
                    return DISCOVERY_PARTICIPANT_DATA_MAX_SIZE;
                }
            , ALIVE, key);

            if(change != nullptr)
            {
                CDRMessage_t aux_msg(change->serializedPayload);

#if __BIG_ENDIAN__
                change->serializedPayload.encapsulation = (uint16_t)PL_CDR_BE;
                aux_msg.msg_endian = BIGEND;
#else
                change->serializedPayload.encapsulation = (uint16_t)PL_CDR_LE;
                aux_msg.msg_endian =  LITTLEEND;
#endif

                if (proxy_data_copy.writeToCDRMessage(&aux_msg, true))
                {
                    change->serializedPayload.length = (uint16_t)aux_msg.length;

                   mp_PDPWriterHistory->add_change(change, wparams);
                }
                else
                {
                    logError(RTPS_PDP, "Cannot serialize ParticipantProxyData.");
                }
            }
        }

    }
    else
    {
        this->mp_mutex->lock();
        ParticipantProxyData proxy_data_copy(*getLocalParticipantProxyData());
        this->mp_mutex->unlock();

        if(mp_PDPWriterHistory->getHistorySize() > 0)
            mp_PDPWriterHistory->remove_min_change();
        change = mp_PDPWriter->new_change([]() -> uint32_t
            {
                return DISCOVERY_PARTICIPANT_DATA_MAX_SIZE;
            }
        , NOT_ALIVE_DISPOSED_UNREGISTERED, getLocalParticipantProxyData()->m_key);

        if(change != nullptr)
        {
            CDRMessage_t aux_msg(change->serializedPayload);

#if __BIG_ENDIAN__
            change->serializedPayload.encapsulation = (uint16_t)PL_CDR_BE;
            aux_msg.msg_endian = BIGEND;
#else
            change->serializedPayload.encapsulation = (uint16_t)PL_CDR_LE;
            aux_msg.msg_endian =  LITTLEEND;
#endif

            if (proxy_data_copy.writeToCDRMessage(&aux_msg, true))
            {
                change->serializedPayload.length = (uint16_t)aux_msg.length;

                mp_PDPWriterHistory->add_change(change, wparams);
            }
            else
            {
                logError(RTPS_PDP, "Cannot serialize ParticipantProxyData.");
            }
        }
    }

}

void PDP::stopParticipantAnnouncement()
{
    resend_participant_info_event_->cancel_timer();
}

void PDP::resetParticipantAnnouncement()
{
    resend_participant_info_event_->restart_timer();
}

bool PDP::has_reader_proxy_data(const GUID_t& reader)
{
    std::lock_guard<std::recursive_mutex> guardPDP(*this->mp_mutex);
    for (std::shared_ptr<ParticipantProxyData> & pit : participant_proxies_)
    {
        if (pit->m_guid.guidPrefix == reader.guidPrefix)
        {
            std::lock_guard<std::recursive_mutex> lock(pit->ppd_mutex_);

            for (ReaderProxyData* rit : pit->m_readers)
            {
                if (rit->guid() == reader)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool PDP::lookupReaderProxyData(const GUID_t& reader, ReaderProxyData& rdata)
{
    std::lock_guard<std::recursive_mutex> guardPDP(*this->mp_mutex);
    for (std::shared_ptr<ParticipantProxyData> & pit : participant_proxies_)
    {
        if (pit->m_guid.guidPrefix == reader.guidPrefix)
        {
            std::lock_guard<std::recursive_mutex> lock(pit->ppd_mutex_);

            for (ReaderProxyData* rit : pit->m_readers)
            {
                if (rit->guid() == reader)
                {
                    rdata.copy(rit);
                    return true;
                }
            }
        }
    }
    return false;
}

bool PDP::has_writer_proxy_data(const GUID_t& writer)
{
    std::lock_guard<std::recursive_mutex> guardPDP(*this->mp_mutex);
    for (std::shared_ptr<ParticipantProxyData> & pit : participant_proxies_)
    {
        if (pit->m_guid.guidPrefix == writer.guidPrefix)
        {
            std::lock_guard<std::recursive_mutex> lock(pit->ppd_mutex_);

            for (WriterProxyData* wit : pit->m_writers)
            {
                if (wit->guid() == writer)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool PDP::lookupWriterProxyData(const GUID_t& writer, WriterProxyData& wdata)
{
    std::lock_guard<std::recursive_mutex> guardPDP(*this->mp_mutex);
    for (std::shared_ptr<ParticipantProxyData> & pit : participant_proxies_)
    {
        if (pit->m_guid.guidPrefix == writer.guidPrefix)
        {
            std::lock_guard<std::recursive_mutex> lock(pit->ppd_mutex_);

            for (WriterProxyData* wit : pit->m_writers)
            {
                if (wit->guid() == writer)
                {
                    wdata.copy(wit);
                    return true;
                }
            }
        }
    }
    return false;
}

bool PDP::removeReaderProxyData(const GUID_t& reader_guid)
{
    logInfo(RTPS_PDP, "Removing reader proxy data " << reader_guid);
    std::lock_guard<std::recursive_mutex> guardPDP(*this->mp_mutex);

    for (std::shared_ptr<ParticipantProxyData>& pit : participant_proxies_)
    {
        if (pit->m_guid.guidPrefix == reader_guid.guidPrefix)
        {
            std::unique_lock<std::recursive_mutex> lock(pit->ppd_mutex_);

            for (ReaderProxyData* rit : pit->m_readers)
            {
                if (rit->guid() == reader_guid)
                {
                    mp_EDP->unpairReaderProxy(pit->m_guid, reader_guid);

                    RTPSParticipantListener* listener = mp_RTPSParticipant->getListener();
                    if (listener)
                    {
                        ReaderDiscoveryInfo info(std::move(*rit));
                        info.status = ReaderDiscoveryInfo::REMOVED_READER;
                        listener->onReaderDiscovery(mp_RTPSParticipant->getUserRTPSParticipant(), std::move(info));
                    }

                    // Clear reader proxy data and move to pool in order to allow reuse
                    rit->clear();
                    pit->m_readers.remove(rit);

                    lock.unlock();
                    std::lock_guard<std::recursive_mutex> pool_lock(PDP::pool_mutex_);
                    reader_proxies_pool_.push_back(rit);
                    return true;
                }
            }
        }
    }

    return false;
}

bool PDP::removeWriterProxyData(const GUID_t& writer_guid)
{
    logInfo(RTPS_PDP, "Removing writer proxy data " << writer_guid);
    std::lock_guard<std::recursive_mutex> guardPDP(*this->mp_mutex);

    for (std::shared_ptr<ParticipantProxyData>& pit : participant_proxies_)
    {
        if (pit->m_guid.guidPrefix == writer_guid.guidPrefix)
        {
            std::unique_lock<std::recursive_mutex> lock(pit->ppd_mutex_);

            for (WriterProxyData* wit : pit->m_writers)
            {
                if (wit->guid() == writer_guid)
                {
                    mp_EDP->unpairWriterProxy(pit->m_guid, writer_guid);

                    RTPSParticipantListener* listener = mp_RTPSParticipant->getListener();
                    if (listener)
                    {
                        WriterDiscoveryInfo info(std::move(*wit));
                        info.status = WriterDiscoveryInfo::REMOVED_WRITER;
                        listener->onWriterDiscovery(mp_RTPSParticipant->getUserRTPSParticipant(), std::move(info));
                    }

                    // Clear writer proxy data and move to pool in order to allow reuse
                    wit->clear();
                    pit->m_writers.remove(wit);
                    lock.unlock();

                    std::lock_guard<std::recursive_mutex> pool_lock(pool_mutex_);
                    writer_proxies_pool_.push_back(wit);
                    return true;
                }
            }
        }
    }

    return false;
}

bool PDP::lookup_participant_name(
        const GUID_t& guid,
        string_255& name)
{
    std::lock_guard<std::recursive_mutex> guardPDP(*this->mp_mutex);
    for (std::shared_ptr<ParticipantProxyData>& pit : participant_proxies_)
    {
        if (pit->m_guid == guid)
        {
            name = pit->m_participantName;
            return true;
        }
    }
    return false;
}

bool PDP::lookup_participant_key(
        const GUID_t& participant_guid,
        InstanceHandle_t& key)
{
    std::lock_guard<std::recursive_mutex> guardPDP(*this->mp_mutex);
    for (std::shared_ptr<ParticipantProxyData>& pit : participant_proxies_)
    {
        if (pit->m_guid == participant_guid)
        {
            key = pit->m_key;
            return true;
        }
    }
    return false;
}

ReaderProxyData* PDP::addReaderProxyData(
        const GUID_t& reader_guid,
        GUID_t& participant_guid,
        std::function<bool(ReaderProxyData*, bool, const ParticipantProxyData&)> initializer_func)
{
    logInfo(RTPS_PDP, "Adding reader proxy data " << reader_guid);
    ReaderProxyData* ret_val = nullptr;

    std::lock_guard<std::recursive_mutex> guardPDP(*this->mp_mutex);

    for(std::shared_ptr<ParticipantProxyData>& pit : participant_proxies_)
    {
        if(pit->m_guid.guidPrefix == reader_guid.guidPrefix)
        {
            std::unique_lock<std::recursive_mutex> ppd_lock(pit->ppd_mutex_);

            // Copy participant data to be used outside.
            participant_guid = pit->m_guid;

            // Check that it is not already there:
            for(ReaderProxyData* rit : pit->m_readers)
            {
                if(rit->guid().entityId == reader_guid.entityId)
                {
                    if (!initializer_func(rit, true, *pit))
                    {
                        return nullptr;
                    }

                    ret_val = rit;

                    RTPSParticipantListener* listener = mp_RTPSParticipant->getListener();
                    if(listener)
                    {
                        ReaderDiscoveryInfo info(*ret_val);
                        info.status = ReaderDiscoveryInfo::CHANGED_QOS_READER;
                        listener->onReaderDiscovery(mp_RTPSParticipant->getUserRTPSParticipant(), std::move(info));
                    }

                    ppd_lock.release(); // If succeeds returns with the lock
                    return ret_val;
                }
            }

            std::lock_guard<std::recursive_mutex> pool_lock(pool_mutex_);

            // Try to take one entry from the pool
            if (reader_proxies_pool_.empty())
            {
                size_t max_proxies = reader_proxies_pool_.max_size();
                if (reader_proxies_number_ < max_proxies)
                {
                    // Pool is empty but limit has not been reached, so we create a new entry.
                    ++reader_proxies_number_;
                    ret_val = new ReaderProxyData(
                        mp_RTPSParticipant->getAttributes().allocation.locators.max_unicast_locators,
                        mp_RTPSParticipant->getAttributes().allocation.locators.max_multicast_locators);
                }
                else
                {
                    logWarning(RTPS_PDP, "Maximum number of reader proxies (" << max_proxies <<
                        ") reached for participant " << mp_RTPSParticipant->getGuid() << std::endl);
                    return nullptr;
                }
            }
            else
            {
                // Pool is not empty, use entry from pool
                ret_val = reader_proxies_pool_.back();
                reader_proxies_pool_.pop_back();
            }

            // Add to ParticipantProxyData
            ret_val->mutex_guard(&pit->ppd_mutex_);
            pit->m_readers.push_back(ret_val);

            if (!initializer_func(ret_val, false, *pit))
            {
                return nullptr;
            }

            RTPSParticipantListener* listener = mp_RTPSParticipant->getListener();
            if(listener)
            {
                ReaderDiscoveryInfo info(*ret_val);
                info.status = ReaderDiscoveryInfo::DISCOVERED_READER;
                listener->onReaderDiscovery(mp_RTPSParticipant->getUserRTPSParticipant(), std::move(info));
            }

            ppd_lock.release(); // If succeeds returns with the lock
            return ret_val;
        }
    }

    return nullptr;
}

WriterProxyData* PDP::addWriterProxyData(
        const GUID_t& writer_guid,
        GUID_t& participant_guid,
        std::function<bool(WriterProxyData*, bool, const ParticipantProxyData&)> initializer_func)
{
    logInfo(RTPS_PDP, "Adding reader proxy data " << writer_guid);
    WriterProxyData* ret_val = nullptr;

    std::lock_guard<std::recursive_mutex> guardPDP(*this->mp_mutex);

    for (std::shared_ptr<ParticipantProxyData>& pit : participant_proxies_)
    {
        if (pit->m_guid.guidPrefix == writer_guid.guidPrefix)
        {
            std::unique_lock<std::recursive_mutex> ppd_lock(pit->ppd_mutex_);

            // Copy participant data to be used outside.
            participant_guid = pit->m_guid;

            // Check that it is not already there:
            for (WriterProxyData* wit : pit->m_writers)
            {
                if (wit->guid().entityId == writer_guid.entityId)
                {
                    if (!initializer_func(wit, true, *pit))
                    {
                        return nullptr;
                    }

                    ret_val = wit;

                    RTPSParticipantListener* listener = mp_RTPSParticipant->getListener();
                    if (listener)
                    {
                        WriterDiscoveryInfo info(*ret_val);
                        info.status = WriterDiscoveryInfo::CHANGED_QOS_WRITER;
                        listener->onWriterDiscovery(mp_RTPSParticipant->getUserRTPSParticipant(), std::move(info));
                    }

                    ppd_lock.release(); // retval is valid thus we return with proxy locked
                    return ret_val;
                }
            }

            std::lock_guard<std::recursive_mutex> pool_lock(pool_mutex_);

            // Try to take one entry from the pool
            if (writer_proxies_pool_.empty())
            {
                size_t max_proxies = writer_proxies_pool_.max_size();
                if (writer_proxies_number_ < max_proxies)
                {
                    // Pool is empty but limit has not been reached, so we create a new entry.
                    ++writer_proxies_number_;
                    ret_val = new WriterProxyData(
                        mp_RTPSParticipant->getAttributes().allocation.locators.max_unicast_locators,
                        mp_RTPSParticipant->getAttributes().allocation.locators.max_multicast_locators);
                }
                else
                {
                    logWarning(RTPS_PDP, "Maximum number of writer proxies (" << max_proxies <<
                        ") reached for participant " << mp_RTPSParticipant->getGuid() << std::endl);
                    return nullptr;
                }
            }
            else
            {
                // Pool is not empty, use entry from pool
                ret_val = writer_proxies_pool_.back();
                writer_proxies_pool_.pop_back();
            }

            // Add to ParticipantProxyData
            ret_val->mutex_guard(&pit->ppd_mutex_);
            pit->m_writers.push_back(ret_val);

            if (!initializer_func(ret_val, false, *pit))
            {
                return nullptr;
            }

            RTPSParticipantListener* listener = mp_RTPSParticipant->getListener();
            if (listener)
            {
                WriterDiscoveryInfo info(*ret_val);
                info.status = WriterDiscoveryInfo::DISCOVERED_WRITER;
                listener->onWriterDiscovery(mp_RTPSParticipant->getUserRTPSParticipant(), std::move(info));
            }

            ppd_lock.release(); // retval is valid thus we return with proxy locked
            return ret_val;
        }
    }

    return nullptr;
}

bool PDP::remove_remote_participant(
        const GUID_t& partGUID,
        ParticipantDiscoveryInfo::DISCOVERY_STATUS reason)
{
    GUID_t local = getLocalParticipantProxyData()->m_guid;

    if ( partGUID == local )
    {   // avoid removing our own data
        return false;
    }

    logInfo(RTPS_PDP,partGUID );
    std::shared_ptr<ParticipantProxyData> pdata = nullptr;

    //Remove it from our vector or RTPSParticipantProxies:
    std::unique_lock<std::recursive_mutex> pdp_lock(*mp_mutex);

    for(ResourceLimitedVector<std::shared_ptr<ParticipantProxyData>>::iterator pit = participant_proxies_.begin();
            pit!=participant_proxies_.end();++pit)
    {
        if((*pit)->m_guid == partGUID)
        {
            pdata = *pit;
            participant_proxies_.erase(pit);
            break;
        }
    }

    if(!pdata)
    {
        // nothing to remove
        return false;
    }

    // proper order of acquisition: PDP -> ppd
    std::unique_lock<std::recursive_mutex> ppd_lock(pdata->ppd_mutex_);
    pdp_lock.unlock();

    if(mp_EDP!=nullptr)
    {
        std::list<GUID_t> to_removal_;

        RTPSParticipantListener* listener = mp_RTPSParticipant->getListener();

        for(ReaderProxyData* rit : pdata->m_readers)
        {
            GUID_t reader_guid(rit->guid());
            if (reader_guid != c_Guid_Unknown)
            {
                to_removal_.push_back(reader_guid);

                if (listener)
                {
                    ReaderDiscoveryInfo info(*rit);
                    info.status = ReaderDiscoveryInfo::REMOVED_READER;
                    listener->onReaderDiscovery(mp_RTPSParticipant->getUserRTPSParticipant(), std::move(info));
                }
            }
        }

        ppd_lock.unlock(); // proper order of adquisition EDP reader -> ppd
        for(GUID_t & reader_guid : to_removal_)
        {
            mp_EDP->unpairReaderProxy(partGUID, reader_guid);
        }
        to_removal_.clear();
        ppd_lock.lock();

        for(WriterProxyData* wit : pdata->m_writers)
        {
            GUID_t writer_guid(wit->guid());
            if (writer_guid != c_Guid_Unknown)
            {
                to_removal_.push_back(writer_guid);

                if (listener)
                {
                    WriterDiscoveryInfo info(*wit);
                    info.status = WriterDiscoveryInfo::REMOVED_WRITER;
                    listener->onWriterDiscovery(mp_RTPSParticipant->getUserRTPSParticipant(), std::move(info));
                }
            }
        }

        ppd_lock.unlock(); // proper order of adquisition EDP reader -> ppd
        for(GUID_t & writer_guid : to_removal_)
        {
            mp_EDP->unpairWriterProxy(partGUID, writer_guid);
        }
        to_removal_.clear();
        ppd_lock.lock();

    }

    if(mp_builtin->mp_WLP != nullptr)
        this->mp_builtin->mp_WLP->removeRemoteEndpoints(pdata.get());
    mp_EDP->removeRemoteEndpoints(pdata.get());
    removeRemoteEndpoints(pdata.get());

#if HAVE_SECURITY
    mp_builtin->mp_participantImpl->security_manager().remove_participant(*pdata);
#endif

    this->mp_PDPReaderHistory->getMutex()->lock();
    for(std::vector<CacheChange_t*>::iterator it=this->mp_PDPReaderHistory->changesBegin();
            it!=this->mp_PDPReaderHistory->changesEnd();++it)
    {
        if((*it)->instanceHandle == pdata->m_key)
        {
            this->mp_PDPReaderHistory->remove_change(*it);
            break;
        }
    }
    this->mp_PDPReaderHistory->getMutex()->unlock();

    auto listener =  mp_RTPSParticipant->getListener();
    if (listener != nullptr)
    {
        std::lock_guard<std::mutex> lock(callback_mtx_);
        ParticipantDiscoveryInfo info(*pdata);
        info.status = reason;
        listener->onParticipantDiscovery(mp_RTPSParticipant->getUserRTPSParticipant(), std::move(info));
    }

    PDP::pool_mutex_.lock();

    // Return reader proxy objects to pool
    for (ReaderProxyData* rit : pdata->m_readers)
    {
        reader_proxies_pool_.push_back(rit);
    }
    pdata->m_readers.clear();

    // Return writer proxy objects to pool
    for (WriterProxyData* wit : pdata->m_writers)
    {
        writer_proxies_pool_.push_back(wit);
    }
    pdata->m_writers.clear();

    PDP::pool_mutex_.unlock();

    // Cancel lease event
    if (pdata->lease_duration_event != nullptr)
    {
        pdata->lease_callback_.remove_listener(local.guidPrefix);
    }

    return true;
 
}

const BuiltinAttributes& PDP::builtin_attributes() const
{
    return mp_builtin->m_att;
}

void PDP::assert_remote_participant_liveliness(
        const GuidPrefix_t& remote_guid)
{
    std::lock_guard<std::recursive_mutex> guardPDP(*this->mp_mutex);

    for (std::shared_ptr<ParticipantProxyData>& it : this->participant_proxies_)
    {
        if(it->m_guid.guidPrefix == remote_guid)
        {
            // TODO Ricardo: Study if isAlive attribute is necessary.
            it->isAlive = true;
            it->assert_liveliness();
            break;
        }
    }
}

CDRMessage_t PDP::get_participant_proxy_data_serialized(Endianness_t endian)
{
    std::lock_guard<std::recursive_mutex> guardPDP(*this->mp_mutex);
    CDRMessage_t cdr_msg;
    cdr_msg.msg_endian = endian;

    if (!getLocalParticipantProxyData()->writeToCDRMessage(&cdr_msg, false))
    {
        cdr_msg.pos = 0;
        cdr_msg.length = 0;
    }

    return cdr_msg;
}


void PDP::check_remote_participant_liveliness(
        ParticipantProxyData* remote_participant)
{
    std::unique_lock<std::recursive_mutex> guard(*mp_mutex);

    if(GUID_t::unknown() != remote_participant->m_guid)
    {
        // Check last received message's time_point plus lease duration time doesn't overcome now().
        // If overcame, remove participant.
        auto now = std::chrono::steady_clock::now();
        auto real_lease_tm = remote_participant->last_received_message_tm() +
                std::chrono::microseconds(TimeConv::Duration_t2MicroSecondsInt64(remote_participant->m_leaseDuration));
        if (now > real_lease_tm)
        {
            guard.unlock();
            remove_remote_participant(remote_participant->m_guid, ParticipantDiscoveryInfo::DROPPED_PARTICIPANT);
            return;
        }

        // Calculate next trigger.
        auto next_trigger = real_lease_tm - now;
        remote_participant->lease_duration_event->update_interval_millisec(
                (double)std::chrono::duration_cast<std::chrono::milliseconds>(next_trigger).count());
        remote_participant->lease_duration_event->restart_timer();
    }
}

void PDP::set_next_announcement_interval()
{
    if (initial_announcements_.count > 0)
    {
        --initial_announcements_.count;
        resend_participant_info_event_->update_interval(initial_announcements_.period);
    }
    else
    {
        resend_participant_info_event_->update_interval(m_discovery.discovery_config.leaseDuration_announcementperiod);
    }
}

void PDP::set_initial_announcement_interval()
{
    if ((initial_announcements_.count > 0) && (initial_announcements_.period <= c_TimeZero))
    {
        // Force a small interval (1ms) between initial announcements
        logWarning(RTPS_PDP, "Initial announcement period is not strictly positive. Changing to 1ms.");
        initial_announcements_.period = { 0, 1000000 };
    }
    set_next_announcement_interval();
}

// TODO: Iker. Participant allocation attributes SHOULD be moved to the library attributes in the future if we
// share the discovery data.

/*static*/
void PDP::initialize_or_update_pool_allocation(const RTPSParticipantAllocationAttributes& allocation)
{
    std::lock_guard<std::recursive_mutex> lock(pool_mutex_);

    if(!pdp_counter_++)
    {
        event_thr_.init_thread();
    }

    participant_proxies_pool_.reserve(allocation.participants.initial);

    if( participant_proxies_number_ < allocation.participants.initial )
    {
        for (size_t i = participant_proxies_number_ ; i < allocation.participants.initial; ++i)
        {
            participant_proxies_pool_.push_back(new ParticipantProxyData(allocation));
        }

        participant_proxies_number_ = allocation.participants.initial;
    }

    // If max_unicast or max_multicast locators changes from participant config to participant config
    // then Reader and Writer proxies will end up with different allocated storage. See Iker's TODO above.
    size_t max_unicast_locators = allocation.locators.max_unicast_locators;
    size_t max_multicast_locators = allocation.locators.max_multicast_locators;

    reader_proxies_pool_.reserve(allocation.total_readers().initial);

    if( reader_proxies_number_ < allocation.total_readers().initial )
    {
        for (size_t i = reader_proxies_number_ ; i < allocation.total_readers().initial; ++i)
        {
            reader_proxies_pool_.push_back(new ReaderProxyData(max_unicast_locators, max_multicast_locators));
        }

        reader_proxies_number_ = allocation.total_readers().initial; 
    }

    writer_proxies_pool_.reserve(allocation.total_writers().initial);

    if( writer_proxies_number_ < allocation.total_writers().initial )
    {
        for (size_t i = writer_proxies_number_ ; i < allocation.total_writers().initial; ++i)
        {
            writer_proxies_pool_.push_back(new WriterProxyData(max_unicast_locators, max_multicast_locators));
        }
        
        writer_proxies_number_ = allocation.total_writers().initial;
    }
}

/*static*/
void PDP::remove_pool_resources()
{
    std::lock_guard<std::recursive_mutex> lock(pool_mutex_);

    if(!--pdp_counter_)
    {
        assert(pool_participant_references_.empty());

        for(ParticipantProxyData* it : participant_proxies_pool_)
        {
            delete it;
        }

        for(ReaderProxyData* it : reader_proxies_pool_)
        {
            delete it;
        }

        for(WriterProxyData* it : writer_proxies_pool_)
        {
            delete it;
        }


    }
}

/*static*/
std::shared_ptr<ParticipantProxyData> PDP::get_from_proxy_pool(const GuidPrefix_t & guid)
{
    std::lock_guard<std::recursive_mutex> lock(pool_mutex_);

    auto it = pool_participant_references_.find(guid);

    if(it == pool_participant_references_.end())
    {
        // nothing there
        return std::shared_ptr<ParticipantProxyData>();
    }

    // recreate shared_ptr from weak
    return it->second.lock();
}

std::shared_ptr<ParticipantProxyData> PDP::get_from_local_proxies(const GuidPrefix_t & guid)
{
    std::lock_guard<std::recursive_mutex> lock(*mp_mutex);

    for(std::shared_ptr<ParticipantProxyData> p : participant_proxies_)
    {
        if(guid == p->m_guid.guidPrefix)
        {
            return p;
        }
    }

    // nothing there
    return std::shared_ptr<ParticipantProxyData>();
}

/*static*/ 
void PDP::return_participant_proxy_to_pool(ParticipantProxyData * p)
{
    assert(p != nullptr);

    GUID_t guid;

    {
        std::lock_guard<std::recursive_mutex> lock(p->ppd_mutex_);
        guid = p->m_guid;
        p->clear();
    }

    if( guid != c_Guid_Unknown)
    {
        std::lock_guard<std::recursive_mutex> lock(PDP::pool_mutex_);

        // if its a pool managed object should be included into the map
        assert(PDP::pool_participant_references_.find(guid.guidPrefix)
            != PDP::pool_participant_references_.end());

        PDP::pool_participant_references_.erase(guid.guidPrefix);
        PDP::participant_proxies_pool_.push_back(p);
    }
}

} /* namespace rtps */
} /* namespace fastrtps */
} /* namespace eprosima */
