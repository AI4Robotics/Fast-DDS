// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef TYPES_DYNAMIC_TYPE_BUILDER_H
#define TYPES_DYNAMIC_TYPE_BUILDER_H

#include <fastdds/dds/log/Log.hpp>
#include <fastrtps/types/TypeDescriptor.h>
#include <fastrtps/utils/custom_allocators.hpp>

namespace eprosima {
namespace fastrtps {
namespace types {

class AnnotationDescriptor;
class TypeDescriptor;
class MemberDescriptor;
class DynamicType;
class DynamicTypeMember;

class DynamicTypeBuilder
    : public TypeDescriptor
    , public std::enable_shared_from_this<DynamicTypeBuilder>
{
    using builder_allocator = detail::BuilderAllocator<DynamicType, DynamicTypeBuilder, false>;

    friend builder_allocator;

    static void after_construction(DynamicType* b);

    static void before_destruction(DynamicType* b);

    // Only create objects from the associated factory
    struct use_the_create_method
    {
        explicit use_the_create_method() = default;
    };

    MemberId current_member_id_ = 0;

    bool check_union_configuration(
            const MemberDescriptor& descriptor);

    using TypeDescriptor::exists_member_by_name;
    using TypeDescriptor::exists_member_by_id;

    void clear();

    DynamicTypeBuilder(const DynamicTypeBuilder&) = default;
    DynamicTypeBuilder(DynamicTypeBuilder&&) = delete;
    DynamicTypeBuilder& operator=(const DynamicTypeBuilder&) = default;
    DynamicTypeBuilder& operator=(DynamicTypeBuilder&&) = delete;

    const TypeDescriptor& get_type_descriptor() const
    {
        return static_cast<const TypeDescriptor&>(*this);
    }

public:

    // TODO Barro: remove this from public
    //! This method only adds an empty element to the members collection with the right index
    member_iterator add_empty_member(
            uint32_t index,
            const std::string& name);
public:

    DynamicTypeBuilder(
            use_the_create_method);

    DynamicTypeBuilder(
            use_the_create_method,
            const DynamicTypeBuilder& builder);

    DynamicTypeBuilder(
            use_the_create_method,
            const TypeDescriptor& descriptor);

    ~DynamicTypeBuilder() = default;

    friend class DynamicTypeBuilderFactory;

    RTPS_DllAPI bool equals(
            const DynamicType& other) const;

    using TypeDescriptor::get_annotation;

    using TypeDescriptor::get_annotation_count;

    using TypeDescriptor::get_descriptor;

    // TODO: doxygen
    RTPS_DllAPI ReturnCode_t add_member(
            const MemberDescriptor& descriptor) noexcept;

    RTPS_DllAPI ReturnCode_t add_member(
            MemberDescriptor&& descriptor) noexcept;

    template<typename... Ts>
    ReturnCode_t add_member(Ts&&... Args) noexcept
    {
        return add_member(MemberDescriptor(std::forward<Ts>(Args)...));
    }

    using AnnotationManager::apply_annotation;

    // TODO: doxygen
    template<typename... Ts>
    ReturnCode_t apply_annotation_to_member(
            MemberId id,
            Ts&&... Args)
    {
        auto it = member_by_id_.find(id);
        if (it != member_by_id_.end())
        {
            it->second->apply_annotation(std::forward<Ts>(Args)...);
            return ReturnCode_t::RETCODE_OK;
        }
        else
        {
            EPROSIMA_LOG_ERROR(DYN_TYPES, "Error applying annotation to member. MemberId not found.");
            return ReturnCode_t::RETCODE_BAD_PARAMETER;
        }
    }

    RTPS_DllAPI DynamicType_ptr build() const;

    RTPS_DllAPI ReturnCode_t copy_from(
            const DynamicTypeBuilder* other);

    using TypeDescriptor::get_all_members;

    using TypeDescriptor::get_all_members_by_name;

    using TypeDescriptor::get_name;

    using TypeDescriptor::get_member_id_by_name;

    using TypeDescriptor::is_consistent;

    bool is_discriminator_type() const;

    using TypeDescriptor::set_name;

    using TypeDescriptor::set_base_type;
};

} // namespace types
} // namespace fastrtps
} // namespace eprosima

#endif // TYPES_DYNAMIC_TYPE_BUILDER_H
