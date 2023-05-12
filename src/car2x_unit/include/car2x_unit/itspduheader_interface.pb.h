// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: itspduheader_interface.proto

#ifndef PROTOBUF_INCLUDED_itspduheader_5finterface_2eproto
#define PROTOBUF_INCLUDED_itspduheader_5finterface_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006001
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_itspduheader_5finterface_2eproto 

namespace protobuf_itspduheader_5finterface_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_itspduheader_5finterface_2eproto
class ItsPduHeader;
class ItsPduHeaderDefaultTypeInternal;
extern ItsPduHeaderDefaultTypeInternal _ItsPduHeader_default_instance_;
namespace google {
namespace protobuf {
template<> ::ItsPduHeader* Arena::CreateMaybeMessage<::ItsPduHeader>(Arena*);
}  // namespace protobuf
}  // namespace google

// ===================================================================

class ItsPduHeader : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:ItsPduHeader) */ {
 public:
  ItsPduHeader();
  virtual ~ItsPduHeader();

  ItsPduHeader(const ItsPduHeader& from);

  inline ItsPduHeader& operator=(const ItsPduHeader& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  ItsPduHeader(ItsPduHeader&& from) noexcept
    : ItsPduHeader() {
    *this = ::std::move(from);
  }

  inline ItsPduHeader& operator=(ItsPduHeader&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const ItsPduHeader& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ItsPduHeader* internal_default_instance() {
    return reinterpret_cast<const ItsPduHeader*>(
               &_ItsPduHeader_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(ItsPduHeader* other);
  friend void swap(ItsPduHeader& a, ItsPduHeader& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ItsPduHeader* New() const final {
    return CreateMaybeMessage<ItsPduHeader>(NULL);
  }

  ItsPduHeader* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<ItsPduHeader>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const ItsPduHeader& from);
  void MergeFrom(const ItsPduHeader& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(ItsPduHeader* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required uint32 protocol_version = 1;
  bool has_protocol_version() const;
  void clear_protocol_version();
  static const int kProtocolVersionFieldNumber = 1;
  ::google::protobuf::uint32 protocol_version() const;
  void set_protocol_version(::google::protobuf::uint32 value);

  // required uint32 message_id = 2;
  bool has_message_id() const;
  void clear_message_id();
  static const int kMessageIdFieldNumber = 2;
  ::google::protobuf::uint32 message_id() const;
  void set_message_id(::google::protobuf::uint32 value);

  // required uint32 station_id = 3;
  bool has_station_id() const;
  void clear_station_id();
  static const int kStationIdFieldNumber = 3;
  ::google::protobuf::uint32 station_id() const;
  void set_station_id(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:ItsPduHeader)
 private:
  void set_has_protocol_version();
  void clear_has_protocol_version();
  void set_has_message_id();
  void clear_has_message_id();
  void set_has_station_id();
  void clear_has_station_id();

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::uint32 protocol_version_;
  ::google::protobuf::uint32 message_id_;
  ::google::protobuf::uint32 station_id_;
  friend struct ::protobuf_itspduheader_5finterface_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ItsPduHeader

// required uint32 protocol_version = 1;
inline bool ItsPduHeader::has_protocol_version() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ItsPduHeader::set_has_protocol_version() {
  _has_bits_[0] |= 0x00000001u;
}
inline void ItsPduHeader::clear_has_protocol_version() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void ItsPduHeader::clear_protocol_version() {
  protocol_version_ = 0u;
  clear_has_protocol_version();
}
inline ::google::protobuf::uint32 ItsPduHeader::protocol_version() const {
  // @@protoc_insertion_point(field_get:ItsPduHeader.protocol_version)
  return protocol_version_;
}
inline void ItsPduHeader::set_protocol_version(::google::protobuf::uint32 value) {
  set_has_protocol_version();
  protocol_version_ = value;
  // @@protoc_insertion_point(field_set:ItsPduHeader.protocol_version)
}

// required uint32 message_id = 2;
inline bool ItsPduHeader::has_message_id() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ItsPduHeader::set_has_message_id() {
  _has_bits_[0] |= 0x00000002u;
}
inline void ItsPduHeader::clear_has_message_id() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void ItsPduHeader::clear_message_id() {
  message_id_ = 0u;
  clear_has_message_id();
}
inline ::google::protobuf::uint32 ItsPduHeader::message_id() const {
  // @@protoc_insertion_point(field_get:ItsPduHeader.message_id)
  return message_id_;
}
inline void ItsPduHeader::set_message_id(::google::protobuf::uint32 value) {
  set_has_message_id();
  message_id_ = value;
  // @@protoc_insertion_point(field_set:ItsPduHeader.message_id)
}

// required uint32 station_id = 3;
inline bool ItsPduHeader::has_station_id() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void ItsPduHeader::set_has_station_id() {
  _has_bits_[0] |= 0x00000004u;
}
inline void ItsPduHeader::clear_has_station_id() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void ItsPduHeader::clear_station_id() {
  station_id_ = 0u;
  clear_has_station_id();
}
inline ::google::protobuf::uint32 ItsPduHeader::station_id() const {
  // @@protoc_insertion_point(field_get:ItsPduHeader.station_id)
  return station_id_;
}
inline void ItsPduHeader::set_station_id(::google::protobuf::uint32 value) {
  set_has_station_id();
  station_id_ = value;
  // @@protoc_insertion_point(field_set:ItsPduHeader.station_id)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)


// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_itspduheader_5finterface_2eproto
