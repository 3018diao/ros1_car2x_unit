// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: itspduheader_interface.proto

#include "itspduheader_interface.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

class ItsPduHeaderDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<ItsPduHeader>
      _instance;
} _ItsPduHeader_default_instance_;
namespace protobuf_itspduheader_5finterface_2eproto {
static void InitDefaultsItsPduHeader() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::_ItsPduHeader_default_instance_;
    new (ptr) ::ItsPduHeader();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::ItsPduHeader::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_ItsPduHeader =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsItsPduHeader}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_ItsPduHeader.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::ItsPduHeader, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::ItsPduHeader, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::ItsPduHeader, protocol_version_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::ItsPduHeader, message_id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::ItsPduHeader, station_id_),
  0,
  1,
  2,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::ItsPduHeader)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::_ItsPduHeader_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "itspduheader_interface.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\034itspduheader_interface.proto\"P\n\014ItsPdu"
      "Header\022\030\n\020protocol_version\030\001 \002(\r\022\022\n\nmess"
      "age_id\030\002 \002(\r\022\022\n\nstation_id\030\003 \002(\r"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 112);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "itspduheader_interface.proto", &protobuf_RegisterTypes);
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_itspduheader_5finterface_2eproto

// ===================================================================

void ItsPduHeader::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int ItsPduHeader::kProtocolVersionFieldNumber;
const int ItsPduHeader::kMessageIdFieldNumber;
const int ItsPduHeader::kStationIdFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

ItsPduHeader::ItsPduHeader()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_itspduheader_5finterface_2eproto::scc_info_ItsPduHeader.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:ItsPduHeader)
}
ItsPduHeader::ItsPduHeader(const ItsPduHeader& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&protocol_version_, &from.protocol_version_,
    static_cast<size_t>(reinterpret_cast<char*>(&station_id_) -
    reinterpret_cast<char*>(&protocol_version_)) + sizeof(station_id_));
  // @@protoc_insertion_point(copy_constructor:ItsPduHeader)
}

void ItsPduHeader::SharedCtor() {
  ::memset(&protocol_version_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&station_id_) -
      reinterpret_cast<char*>(&protocol_version_)) + sizeof(station_id_));
}

ItsPduHeader::~ItsPduHeader() {
  // @@protoc_insertion_point(destructor:ItsPduHeader)
  SharedDtor();
}

void ItsPduHeader::SharedDtor() {
}

void ItsPduHeader::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* ItsPduHeader::descriptor() {
  ::protobuf_itspduheader_5finterface_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_itspduheader_5finterface_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const ItsPduHeader& ItsPduHeader::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_itspduheader_5finterface_2eproto::scc_info_ItsPduHeader.base);
  return *internal_default_instance();
}


void ItsPduHeader::Clear() {
// @@protoc_insertion_point(message_clear_start:ItsPduHeader)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 7u) {
    ::memset(&protocol_version_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&station_id_) -
        reinterpret_cast<char*>(&protocol_version_)) + sizeof(station_id_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool ItsPduHeader::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:ItsPduHeader)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required uint32 protocol_version = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          set_has_protocol_version();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &protocol_version_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required uint32 message_id = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(16u /* 16 & 0xFF */)) {
          set_has_message_id();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &message_id_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required uint32 station_id = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u /* 24 & 0xFF */)) {
          set_has_station_id();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &station_id_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:ItsPduHeader)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:ItsPduHeader)
  return false;
#undef DO_
}

void ItsPduHeader::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:ItsPduHeader)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required uint32 protocol_version = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(1, this->protocol_version(), output);
  }

  // required uint32 message_id = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(2, this->message_id(), output);
  }

  // required uint32 station_id = 3;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(3, this->station_id(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:ItsPduHeader)
}

::google::protobuf::uint8* ItsPduHeader::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:ItsPduHeader)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required uint32 protocol_version = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(1, this->protocol_version(), target);
  }

  // required uint32 message_id = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(2, this->message_id(), target);
  }

  // required uint32 station_id = 3;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(3, this->station_id(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ItsPduHeader)
  return target;
}

size_t ItsPduHeader::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:ItsPduHeader)
  size_t total_size = 0;

  if (has_protocol_version()) {
    // required uint32 protocol_version = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->protocol_version());
  }

  if (has_message_id()) {
    // required uint32 message_id = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->message_id());
  }

  if (has_station_id()) {
    // required uint32 station_id = 3;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->station_id());
  }

  return total_size;
}
size_t ItsPduHeader::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ItsPduHeader)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (((_has_bits_[0] & 0x00000007) ^ 0x00000007) == 0) {  // All required fields are present.
    // required uint32 protocol_version = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->protocol_version());

    // required uint32 message_id = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->message_id());

    // required uint32 station_id = 3;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->station_id());

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ItsPduHeader::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:ItsPduHeader)
  GOOGLE_DCHECK_NE(&from, this);
  const ItsPduHeader* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const ItsPduHeader>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:ItsPduHeader)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:ItsPduHeader)
    MergeFrom(*source);
  }
}

void ItsPduHeader::MergeFrom(const ItsPduHeader& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ItsPduHeader)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 7u) {
    if (cached_has_bits & 0x00000001u) {
      protocol_version_ = from.protocol_version_;
    }
    if (cached_has_bits & 0x00000002u) {
      message_id_ = from.message_id_;
    }
    if (cached_has_bits & 0x00000004u) {
      station_id_ = from.station_id_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ItsPduHeader::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:ItsPduHeader)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ItsPduHeader::CopyFrom(const ItsPduHeader& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ItsPduHeader)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ItsPduHeader::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000007) != 0x00000007) return false;
  return true;
}

void ItsPduHeader::Swap(ItsPduHeader* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ItsPduHeader::InternalSwap(ItsPduHeader* other) {
  using std::swap;
  swap(protocol_version_, other->protocol_version_);
  swap(message_id_, other->message_id_);
  swap(station_id_, other->station_id_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata ItsPduHeader::GetMetadata() const {
  protobuf_itspduheader_5finterface_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_itspduheader_5finterface_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::ItsPduHeader* Arena::CreateMaybeMessage< ::ItsPduHeader >(Arena* arena) {
  return Arena::CreateInternal< ::ItsPduHeader >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)