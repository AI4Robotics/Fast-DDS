// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file TopicPayloadPoolProxy.hpp
 */

#ifndef DDS_PAYLOADPOOL_IMPL_HPP
#define DDS_PAYLOADPOOL_IMPL_HPP

#include <dds/common/PayloadPool.hpp>

#include <memory>
#include <string>

namespace eprosima {
namespace fastrtps {
namespace dds {

/**
 * Proxy class that adds the topic name to a ITopicPayloadPool, so we can look-up
 * the corresponding entry in the registry when releasing the pool.
 */
class PayloadPool : public IPayloadPool
{

public:

    struct DestructorHelper
    {
        static DestructorHelper& instance()
        {
            static DestructorHelper singleton;
            return singleton;
        }

        void increment()
        {
            ++num_objects_destroyed;
        }

        size_t get()
        {
            return num_objects_destroyed;
        }

    private:

        size_t num_objects_destroyed = 0u;
    };

    PayloadPool(
            const std::string& topic_name,
            const BasicPoolConfig& config)
        : topic_name_(topic_name)
        , policy_(config.memory_policy)
        , inner_pool_(PayloadPool::get(config))
    {
    }

    ~PayloadPool()
    {
        DestructorHelper::instance().increment();
    }

    const std::string& topic_name() const
    {
        return topic_name_;
    }

    MemoryManagementPolicy_t memory_policy() const
    {
        return policy_;
    }

    bool get_payload(
            uint32_t size,
            CacheChange_t& cache_change) override
    {
        return inner_pool_->get_payload(size, cache_change);
    }

    bool get_payload(
            SerializedPayload_t& data,
            IPayloadPool*& data_owner,
            CacheChange_t& cache_change) override
    {
        return inner_pool_->get_payload(data, data_owner, cache_change);
    }

    bool release_payload(
            CacheChange_t& cache_change) override
    {
        return inner_pool_->release_payload(cache_change);
    }

    bool reserve_history(
            const PoolConfig& config,
            bool is_reader) override
    {
        return inner_pool_->reserve_history(config, is_reader);
    }

    bool release_history(
            const PoolConfig& config,
            bool is_reader) override
    {
        return inner_pool_->release_history(config, is_reader);
    }

    size_t payload_pool_allocated_size() const override
    {
        return inner_pool_->payload_pool_allocated_size();
    }

    size_t payload_pool_available_size() const override
    {
        return inner_pool_->payload_pool_available_size();
    }

private:

    std::string topic_name_;
    MemoryManagementPolicy_t policy_;
    std::unique_ptr<IPayloadPool> inner_pool_;

};

}  // namespace detail
}  // namespace rtps
}  // namespace fastrtps
}  // namespace eprosima

#endif  // DDS_PAYLOADPOOL_IMPL_HPP
