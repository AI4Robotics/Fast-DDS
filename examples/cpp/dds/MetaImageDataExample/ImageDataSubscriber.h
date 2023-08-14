// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file ImageDataSubscriber.h
 *
 */

#ifndef IMAGEDATASUBSCRIBER_H_
#define IMAGEDATASUBSCRIBER_H_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>

#include <fastdds/dds/core/status/BaseStatus.hpp>
#include <fastdds/dds/core/status/SubscriptionMatchedStatus.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "ImageData.h"

class ImageDataSubscriber : public eprosima::fastdds::dds::DataReaderListener
{
public:

    ImageDataSubscriber();

    virtual ~ImageDataSubscriber();

    //!Initialize the subscriber
    bool init();

    //!RUN the subscriber
    void run();

private:

    void on_data_available(
            eprosima::fastdds::dds::DataReader* reader) override;

    void on_subscription_matched(
            eprosima::fastdds::dds::DataReader* reader,
            const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override;

    void on_sample_lost(
            eprosima::fastdds::dds::DataReader* reader,
            const eprosima::fastdds::dds::SampleLostStatus& status) override;

    eprosima::fastdds::dds::DomainParticipant* participant_;

    eprosima::fastdds::dds::Subscriber* subscriber_;

    eprosima::fastdds::dds::Topic* topic_;

    eprosima::fastdds::dds::DataReader* reader_;

    eprosima::fastdds::dds::TypeSupport type_;

    ImageDataMsg msg_;

    int matched_;

    uint32_t received_samples_;

    uint32_t lost_samples_;

    static std::atomic<bool> running_;

    static std::mutex mtx_;

    static std::condition_variable cv_;
};

#endif /* IMAGEDATASUBSCRIBER_H_ */
