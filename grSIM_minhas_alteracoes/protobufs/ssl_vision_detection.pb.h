// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ssl_vision_detection.proto

#ifndef PROTOBUF_INCLUDED_ssl_5fvision_5fdetection_2eproto
#define PROTOBUF_INCLUDED_ssl_5fvision_5fdetection_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_protobuf_ssl_5fvision_5fdetection_2eproto 

namespace protobuf_ssl_5fvision_5fdetection_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[3];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_ssl_5fvision_5fdetection_2eproto
class SSL_DetectionBall;
class SSL_DetectionBallDefaultTypeInternal;
extern SSL_DetectionBallDefaultTypeInternal _SSL_DetectionBall_default_instance_;
class SSL_DetectionFrame;
class SSL_DetectionFrameDefaultTypeInternal;
extern SSL_DetectionFrameDefaultTypeInternal _SSL_DetectionFrame_default_instance_;
class SSL_DetectionRobot;
class SSL_DetectionRobotDefaultTypeInternal;
extern SSL_DetectionRobotDefaultTypeInternal _SSL_DetectionRobot_default_instance_;
namespace google {
namespace protobuf {
template<> ::SSL_DetectionBall* Arena::CreateMaybeMessage<::SSL_DetectionBall>(Arena*);
template<> ::SSL_DetectionFrame* Arena::CreateMaybeMessage<::SSL_DetectionFrame>(Arena*);
template<> ::SSL_DetectionRobot* Arena::CreateMaybeMessage<::SSL_DetectionRobot>(Arena*);
}  // namespace protobuf
}  // namespace google

// ===================================================================

class SSL_DetectionBall : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:SSL_DetectionBall) */ {
 public:
  SSL_DetectionBall();
  virtual ~SSL_DetectionBall();

  SSL_DetectionBall(const SSL_DetectionBall& from);

  inline SSL_DetectionBall& operator=(const SSL_DetectionBall& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  SSL_DetectionBall(SSL_DetectionBall&& from) noexcept
    : SSL_DetectionBall() {
    *this = ::std::move(from);
  }

  inline SSL_DetectionBall& operator=(SSL_DetectionBall&& from) noexcept {
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
  static const SSL_DetectionBall& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SSL_DetectionBall* internal_default_instance() {
    return reinterpret_cast<const SSL_DetectionBall*>(
               &_SSL_DetectionBall_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(SSL_DetectionBall* other);
  friend void swap(SSL_DetectionBall& a, SSL_DetectionBall& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline SSL_DetectionBall* New() const final {
    return CreateMaybeMessage<SSL_DetectionBall>(NULL);
  }

  SSL_DetectionBall* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<SSL_DetectionBall>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const SSL_DetectionBall& from);
  void MergeFrom(const SSL_DetectionBall& from);
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
  void InternalSwap(SSL_DetectionBall* other);
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

  // required float confidence = 1;
  bool has_confidence() const;
  void clear_confidence();
  static const int kConfidenceFieldNumber = 1;
  float confidence() const;
  void set_confidence(float value);

  // optional uint32 area = 2;
  bool has_area() const;
  void clear_area();
  static const int kAreaFieldNumber = 2;
  ::google::protobuf::uint32 area() const;
  void set_area(::google::protobuf::uint32 value);

  // required float x = 3;
  bool has_x() const;
  void clear_x();
  static const int kXFieldNumber = 3;
  float x() const;
  void set_x(float value);

  // required float y = 4;
  bool has_y() const;
  void clear_y();
  static const int kYFieldNumber = 4;
  float y() const;
  void set_y(float value);

  // optional float z = 5;
  bool has_z() const;
  void clear_z();
  static const int kZFieldNumber = 5;
  float z() const;
  void set_z(float value);

  // required float pixel_x = 6;
  bool has_pixel_x() const;
  void clear_pixel_x();
  static const int kPixelXFieldNumber = 6;
  float pixel_x() const;
  void set_pixel_x(float value);

  // required float pixel_y = 7;
  bool has_pixel_y() const;
  void clear_pixel_y();
  static const int kPixelYFieldNumber = 7;
  float pixel_y() const;
  void set_pixel_y(float value);

  // @@protoc_insertion_point(class_scope:SSL_DetectionBall)
 private:
  void set_has_confidence();
  void clear_has_confidence();
  void set_has_area();
  void clear_has_area();
  void set_has_x();
  void clear_has_x();
  void set_has_y();
  void clear_has_y();
  void set_has_z();
  void clear_has_z();
  void set_has_pixel_x();
  void clear_has_pixel_x();
  void set_has_pixel_y();
  void clear_has_pixel_y();

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  float confidence_;
  ::google::protobuf::uint32 area_;
  float x_;
  float y_;
  float z_;
  float pixel_x_;
  float pixel_y_;
  friend struct ::protobuf_ssl_5fvision_5fdetection_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class SSL_DetectionRobot : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:SSL_DetectionRobot) */ {
 public:
  SSL_DetectionRobot();
  virtual ~SSL_DetectionRobot();

  SSL_DetectionRobot(const SSL_DetectionRobot& from);

  inline SSL_DetectionRobot& operator=(const SSL_DetectionRobot& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  SSL_DetectionRobot(SSL_DetectionRobot&& from) noexcept
    : SSL_DetectionRobot() {
    *this = ::std::move(from);
  }

  inline SSL_DetectionRobot& operator=(SSL_DetectionRobot&& from) noexcept {
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
  static const SSL_DetectionRobot& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SSL_DetectionRobot* internal_default_instance() {
    return reinterpret_cast<const SSL_DetectionRobot*>(
               &_SSL_DetectionRobot_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(SSL_DetectionRobot* other);
  friend void swap(SSL_DetectionRobot& a, SSL_DetectionRobot& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline SSL_DetectionRobot* New() const final {
    return CreateMaybeMessage<SSL_DetectionRobot>(NULL);
  }

  SSL_DetectionRobot* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<SSL_DetectionRobot>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const SSL_DetectionRobot& from);
  void MergeFrom(const SSL_DetectionRobot& from);
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
  void InternalSwap(SSL_DetectionRobot* other);
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

  // required float confidence = 1;
  bool has_confidence() const;
  void clear_confidence();
  static const int kConfidenceFieldNumber = 1;
  float confidence() const;
  void set_confidence(float value);

  // optional uint32 robot_id = 2;
  bool has_robot_id() const;
  void clear_robot_id();
  static const int kRobotIdFieldNumber = 2;
  ::google::protobuf::uint32 robot_id() const;
  void set_robot_id(::google::protobuf::uint32 value);

  // required float x = 3;
  bool has_x() const;
  void clear_x();
  static const int kXFieldNumber = 3;
  float x() const;
  void set_x(float value);

  // required float y = 4;
  bool has_y() const;
  void clear_y();
  static const int kYFieldNumber = 4;
  float y() const;
  void set_y(float value);

  // optional float orientation = 5;
  bool has_orientation() const;
  void clear_orientation();
  static const int kOrientationFieldNumber = 5;
  float orientation() const;
  void set_orientation(float value);

  // required float pixel_x = 6;
  bool has_pixel_x() const;
  void clear_pixel_x();
  static const int kPixelXFieldNumber = 6;
  float pixel_x() const;
  void set_pixel_x(float value);

  // required float pixel_y = 7;
  bool has_pixel_y() const;
  void clear_pixel_y();
  static const int kPixelYFieldNumber = 7;
  float pixel_y() const;
  void set_pixel_y(float value);

  // optional float height = 8;
  bool has_height() const;
  void clear_height();
  static const int kHeightFieldNumber = 8;
  float height() const;
  void set_height(float value);

  // @@protoc_insertion_point(class_scope:SSL_DetectionRobot)
 private:
  void set_has_confidence();
  void clear_has_confidence();
  void set_has_robot_id();
  void clear_has_robot_id();
  void set_has_x();
  void clear_has_x();
  void set_has_y();
  void clear_has_y();
  void set_has_orientation();
  void clear_has_orientation();
  void set_has_pixel_x();
  void clear_has_pixel_x();
  void set_has_pixel_y();
  void clear_has_pixel_y();
  void set_has_height();
  void clear_has_height();

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  float confidence_;
  ::google::protobuf::uint32 robot_id_;
  float x_;
  float y_;
  float orientation_;
  float pixel_x_;
  float pixel_y_;
  float height_;
  friend struct ::protobuf_ssl_5fvision_5fdetection_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class SSL_DetectionFrame : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:SSL_DetectionFrame) */ {
 public:
  SSL_DetectionFrame();
  virtual ~SSL_DetectionFrame();

  SSL_DetectionFrame(const SSL_DetectionFrame& from);

  inline SSL_DetectionFrame& operator=(const SSL_DetectionFrame& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  SSL_DetectionFrame(SSL_DetectionFrame&& from) noexcept
    : SSL_DetectionFrame() {
    *this = ::std::move(from);
  }

  inline SSL_DetectionFrame& operator=(SSL_DetectionFrame&& from) noexcept {
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
  static const SSL_DetectionFrame& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SSL_DetectionFrame* internal_default_instance() {
    return reinterpret_cast<const SSL_DetectionFrame*>(
               &_SSL_DetectionFrame_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  void Swap(SSL_DetectionFrame* other);
  friend void swap(SSL_DetectionFrame& a, SSL_DetectionFrame& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline SSL_DetectionFrame* New() const final {
    return CreateMaybeMessage<SSL_DetectionFrame>(NULL);
  }

  SSL_DetectionFrame* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<SSL_DetectionFrame>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const SSL_DetectionFrame& from);
  void MergeFrom(const SSL_DetectionFrame& from);
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
  void InternalSwap(SSL_DetectionFrame* other);
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

  // repeated .SSL_DetectionBall balls = 5;
  int balls_size() const;
  void clear_balls();
  static const int kBallsFieldNumber = 5;
  ::SSL_DetectionBall* mutable_balls(int index);
  ::google::protobuf::RepeatedPtrField< ::SSL_DetectionBall >*
      mutable_balls();
  const ::SSL_DetectionBall& balls(int index) const;
  ::SSL_DetectionBall* add_balls();
  const ::google::protobuf::RepeatedPtrField< ::SSL_DetectionBall >&
      balls() const;

  // repeated .SSL_DetectionRobot robots_yellow = 6;
  int robots_yellow_size() const;
  void clear_robots_yellow();
  static const int kRobotsYellowFieldNumber = 6;
  ::SSL_DetectionRobot* mutable_robots_yellow(int index);
  ::google::protobuf::RepeatedPtrField< ::SSL_DetectionRobot >*
      mutable_robots_yellow();
  const ::SSL_DetectionRobot& robots_yellow(int index) const;
  ::SSL_DetectionRobot* add_robots_yellow();
  const ::google::protobuf::RepeatedPtrField< ::SSL_DetectionRobot >&
      robots_yellow() const;

  // repeated .SSL_DetectionRobot robots_blue = 7;
  int robots_blue_size() const;
  void clear_robots_blue();
  static const int kRobotsBlueFieldNumber = 7;
  ::SSL_DetectionRobot* mutable_robots_blue(int index);
  ::google::protobuf::RepeatedPtrField< ::SSL_DetectionRobot >*
      mutable_robots_blue();
  const ::SSL_DetectionRobot& robots_blue(int index) const;
  ::SSL_DetectionRobot* add_robots_blue();
  const ::google::protobuf::RepeatedPtrField< ::SSL_DetectionRobot >&
      robots_blue() const;

  // required double t_capture = 2;
  bool has_t_capture() const;
  void clear_t_capture();
  static const int kTCaptureFieldNumber = 2;
  double t_capture() const;
  void set_t_capture(double value);

  // required uint32 frame_number = 1;
  bool has_frame_number() const;
  void clear_frame_number();
  static const int kFrameNumberFieldNumber = 1;
  ::google::protobuf::uint32 frame_number() const;
  void set_frame_number(::google::protobuf::uint32 value);

  // required uint32 camera_id = 4;
  bool has_camera_id() const;
  void clear_camera_id();
  static const int kCameraIdFieldNumber = 4;
  ::google::protobuf::uint32 camera_id() const;
  void set_camera_id(::google::protobuf::uint32 value);

  // required double t_sent = 3;
  bool has_t_sent() const;
  void clear_t_sent();
  static const int kTSentFieldNumber = 3;
  double t_sent() const;
  void set_t_sent(double value);

  // @@protoc_insertion_point(class_scope:SSL_DetectionFrame)
 private:
  void set_has_frame_number();
  void clear_has_frame_number();
  void set_has_t_capture();
  void clear_has_t_capture();
  void set_has_t_sent();
  void clear_has_t_sent();
  void set_has_camera_id();
  void clear_has_camera_id();

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::SSL_DetectionBall > balls_;
  ::google::protobuf::RepeatedPtrField< ::SSL_DetectionRobot > robots_yellow_;
  ::google::protobuf::RepeatedPtrField< ::SSL_DetectionRobot > robots_blue_;
  double t_capture_;
  ::google::protobuf::uint32 frame_number_;
  ::google::protobuf::uint32 camera_id_;
  double t_sent_;
  friend struct ::protobuf_ssl_5fvision_5fdetection_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SSL_DetectionBall

// required float confidence = 1;
inline bool SSL_DetectionBall::has_confidence() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SSL_DetectionBall::set_has_confidence() {
  _has_bits_[0] |= 0x00000001u;
}
inline void SSL_DetectionBall::clear_has_confidence() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void SSL_DetectionBall::clear_confidence() {
  confidence_ = 0;
  clear_has_confidence();
}
inline float SSL_DetectionBall::confidence() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionBall.confidence)
  return confidence_;
}
inline void SSL_DetectionBall::set_confidence(float value) {
  set_has_confidence();
  confidence_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionBall.confidence)
}

// optional uint32 area = 2;
inline bool SSL_DetectionBall::has_area() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void SSL_DetectionBall::set_has_area() {
  _has_bits_[0] |= 0x00000002u;
}
inline void SSL_DetectionBall::clear_has_area() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void SSL_DetectionBall::clear_area() {
  area_ = 0u;
  clear_has_area();
}
inline ::google::protobuf::uint32 SSL_DetectionBall::area() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionBall.area)
  return area_;
}
inline void SSL_DetectionBall::set_area(::google::protobuf::uint32 value) {
  set_has_area();
  area_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionBall.area)
}

// required float x = 3;
inline bool SSL_DetectionBall::has_x() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void SSL_DetectionBall::set_has_x() {
  _has_bits_[0] |= 0x00000004u;
}
inline void SSL_DetectionBall::clear_has_x() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void SSL_DetectionBall::clear_x() {
  x_ = 0;
  clear_has_x();
}
inline float SSL_DetectionBall::x() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionBall.x)
  return x_;
}
inline void SSL_DetectionBall::set_x(float value) {
  set_has_x();
  x_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionBall.x)
}

// required float y = 4;
inline bool SSL_DetectionBall::has_y() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void SSL_DetectionBall::set_has_y() {
  _has_bits_[0] |= 0x00000008u;
}
inline void SSL_DetectionBall::clear_has_y() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void SSL_DetectionBall::clear_y() {
  y_ = 0;
  clear_has_y();
}
inline float SSL_DetectionBall::y() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionBall.y)
  return y_;
}
inline void SSL_DetectionBall::set_y(float value) {
  set_has_y();
  y_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionBall.y)
}

// optional float z = 5;
inline bool SSL_DetectionBall::has_z() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void SSL_DetectionBall::set_has_z() {
  _has_bits_[0] |= 0x00000010u;
}
inline void SSL_DetectionBall::clear_has_z() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void SSL_DetectionBall::clear_z() {
  z_ = 0;
  clear_has_z();
}
inline float SSL_DetectionBall::z() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionBall.z)
  return z_;
}
inline void SSL_DetectionBall::set_z(float value) {
  set_has_z();
  z_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionBall.z)
}

// required float pixel_x = 6;
inline bool SSL_DetectionBall::has_pixel_x() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void SSL_DetectionBall::set_has_pixel_x() {
  _has_bits_[0] |= 0x00000020u;
}
inline void SSL_DetectionBall::clear_has_pixel_x() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void SSL_DetectionBall::clear_pixel_x() {
  pixel_x_ = 0;
  clear_has_pixel_x();
}
inline float SSL_DetectionBall::pixel_x() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionBall.pixel_x)
  return pixel_x_;
}
inline void SSL_DetectionBall::set_pixel_x(float value) {
  set_has_pixel_x();
  pixel_x_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionBall.pixel_x)
}

// required float pixel_y = 7;
inline bool SSL_DetectionBall::has_pixel_y() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void SSL_DetectionBall::set_has_pixel_y() {
  _has_bits_[0] |= 0x00000040u;
}
inline void SSL_DetectionBall::clear_has_pixel_y() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void SSL_DetectionBall::clear_pixel_y() {
  pixel_y_ = 0;
  clear_has_pixel_y();
}
inline float SSL_DetectionBall::pixel_y() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionBall.pixel_y)
  return pixel_y_;
}
inline void SSL_DetectionBall::set_pixel_y(float value) {
  set_has_pixel_y();
  pixel_y_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionBall.pixel_y)
}

// -------------------------------------------------------------------

// SSL_DetectionRobot

// required float confidence = 1;
inline bool SSL_DetectionRobot::has_confidence() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SSL_DetectionRobot::set_has_confidence() {
  _has_bits_[0] |= 0x00000001u;
}
inline void SSL_DetectionRobot::clear_has_confidence() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void SSL_DetectionRobot::clear_confidence() {
  confidence_ = 0;
  clear_has_confidence();
}
inline float SSL_DetectionRobot::confidence() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionRobot.confidence)
  return confidence_;
}
inline void SSL_DetectionRobot::set_confidence(float value) {
  set_has_confidence();
  confidence_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionRobot.confidence)
}

// optional uint32 robot_id = 2;
inline bool SSL_DetectionRobot::has_robot_id() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void SSL_DetectionRobot::set_has_robot_id() {
  _has_bits_[0] |= 0x00000002u;
}
inline void SSL_DetectionRobot::clear_has_robot_id() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void SSL_DetectionRobot::clear_robot_id() {
  robot_id_ = 0u;
  clear_has_robot_id();
}
inline ::google::protobuf::uint32 SSL_DetectionRobot::robot_id() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionRobot.robot_id)
  return robot_id_;
}
inline void SSL_DetectionRobot::set_robot_id(::google::protobuf::uint32 value) {
  set_has_robot_id();
  robot_id_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionRobot.robot_id)
}

// required float x = 3;
inline bool SSL_DetectionRobot::has_x() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void SSL_DetectionRobot::set_has_x() {
  _has_bits_[0] |= 0x00000004u;
}
inline void SSL_DetectionRobot::clear_has_x() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void SSL_DetectionRobot::clear_x() {
  x_ = 0;
  clear_has_x();
}
inline float SSL_DetectionRobot::x() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionRobot.x)
  return x_;
}
inline void SSL_DetectionRobot::set_x(float value) {
  set_has_x();
  x_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionRobot.x)
}

// required float y = 4;
inline bool SSL_DetectionRobot::has_y() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void SSL_DetectionRobot::set_has_y() {
  _has_bits_[0] |= 0x00000008u;
}
inline void SSL_DetectionRobot::clear_has_y() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void SSL_DetectionRobot::clear_y() {
  y_ = 0;
  clear_has_y();
}
inline float SSL_DetectionRobot::y() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionRobot.y)
  return y_;
}
inline void SSL_DetectionRobot::set_y(float value) {
  set_has_y();
  y_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionRobot.y)
}

// optional float orientation = 5;
inline bool SSL_DetectionRobot::has_orientation() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void SSL_DetectionRobot::set_has_orientation() {
  _has_bits_[0] |= 0x00000010u;
}
inline void SSL_DetectionRobot::clear_has_orientation() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void SSL_DetectionRobot::clear_orientation() {
  orientation_ = 0;
  clear_has_orientation();
}
inline float SSL_DetectionRobot::orientation() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionRobot.orientation)
  return orientation_;
}
inline void SSL_DetectionRobot::set_orientation(float value) {
  set_has_orientation();
  orientation_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionRobot.orientation)
}

// required float pixel_x = 6;
inline bool SSL_DetectionRobot::has_pixel_x() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void SSL_DetectionRobot::set_has_pixel_x() {
  _has_bits_[0] |= 0x00000020u;
}
inline void SSL_DetectionRobot::clear_has_pixel_x() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void SSL_DetectionRobot::clear_pixel_x() {
  pixel_x_ = 0;
  clear_has_pixel_x();
}
inline float SSL_DetectionRobot::pixel_x() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionRobot.pixel_x)
  return pixel_x_;
}
inline void SSL_DetectionRobot::set_pixel_x(float value) {
  set_has_pixel_x();
  pixel_x_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionRobot.pixel_x)
}

// required float pixel_y = 7;
inline bool SSL_DetectionRobot::has_pixel_y() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void SSL_DetectionRobot::set_has_pixel_y() {
  _has_bits_[0] |= 0x00000040u;
}
inline void SSL_DetectionRobot::clear_has_pixel_y() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void SSL_DetectionRobot::clear_pixel_y() {
  pixel_y_ = 0;
  clear_has_pixel_y();
}
inline float SSL_DetectionRobot::pixel_y() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionRobot.pixel_y)
  return pixel_y_;
}
inline void SSL_DetectionRobot::set_pixel_y(float value) {
  set_has_pixel_y();
  pixel_y_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionRobot.pixel_y)
}

// optional float height = 8;
inline bool SSL_DetectionRobot::has_height() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void SSL_DetectionRobot::set_has_height() {
  _has_bits_[0] |= 0x00000080u;
}
inline void SSL_DetectionRobot::clear_has_height() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void SSL_DetectionRobot::clear_height() {
  height_ = 0;
  clear_has_height();
}
inline float SSL_DetectionRobot::height() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionRobot.height)
  return height_;
}
inline void SSL_DetectionRobot::set_height(float value) {
  set_has_height();
  height_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionRobot.height)
}

// -------------------------------------------------------------------

// SSL_DetectionFrame

// required uint32 frame_number = 1;
inline bool SSL_DetectionFrame::has_frame_number() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void SSL_DetectionFrame::set_has_frame_number() {
  _has_bits_[0] |= 0x00000002u;
}
inline void SSL_DetectionFrame::clear_has_frame_number() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void SSL_DetectionFrame::clear_frame_number() {
  frame_number_ = 0u;
  clear_has_frame_number();
}
inline ::google::protobuf::uint32 SSL_DetectionFrame::frame_number() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionFrame.frame_number)
  return frame_number_;
}
inline void SSL_DetectionFrame::set_frame_number(::google::protobuf::uint32 value) {
  set_has_frame_number();
  frame_number_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionFrame.frame_number)
}

// required double t_capture = 2;
inline bool SSL_DetectionFrame::has_t_capture() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SSL_DetectionFrame::set_has_t_capture() {
  _has_bits_[0] |= 0x00000001u;
}
inline void SSL_DetectionFrame::clear_has_t_capture() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void SSL_DetectionFrame::clear_t_capture() {
  t_capture_ = 0;
  clear_has_t_capture();
}
inline double SSL_DetectionFrame::t_capture() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionFrame.t_capture)
  return t_capture_;
}
inline void SSL_DetectionFrame::set_t_capture(double value) {
  set_has_t_capture();
  t_capture_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionFrame.t_capture)
}

// required double t_sent = 3;
inline bool SSL_DetectionFrame::has_t_sent() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void SSL_DetectionFrame::set_has_t_sent() {
  _has_bits_[0] |= 0x00000008u;
}
inline void SSL_DetectionFrame::clear_has_t_sent() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void SSL_DetectionFrame::clear_t_sent() {
  t_sent_ = 0;
  clear_has_t_sent();
}
inline double SSL_DetectionFrame::t_sent() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionFrame.t_sent)
  return t_sent_;
}
inline void SSL_DetectionFrame::set_t_sent(double value) {
  set_has_t_sent();
  t_sent_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionFrame.t_sent)
}

// required uint32 camera_id = 4;
inline bool SSL_DetectionFrame::has_camera_id() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void SSL_DetectionFrame::set_has_camera_id() {
  _has_bits_[0] |= 0x00000004u;
}
inline void SSL_DetectionFrame::clear_has_camera_id() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void SSL_DetectionFrame::clear_camera_id() {
  camera_id_ = 0u;
  clear_has_camera_id();
}
inline ::google::protobuf::uint32 SSL_DetectionFrame::camera_id() const {
  // @@protoc_insertion_point(field_get:SSL_DetectionFrame.camera_id)
  return camera_id_;
}
inline void SSL_DetectionFrame::set_camera_id(::google::protobuf::uint32 value) {
  set_has_camera_id();
  camera_id_ = value;
  // @@protoc_insertion_point(field_set:SSL_DetectionFrame.camera_id)
}

// repeated .SSL_DetectionBall balls = 5;
inline int SSL_DetectionFrame::balls_size() const {
  return balls_.size();
}
inline void SSL_DetectionFrame::clear_balls() {
  balls_.Clear();
}
inline ::SSL_DetectionBall* SSL_DetectionFrame::mutable_balls(int index) {
  // @@protoc_insertion_point(field_mutable:SSL_DetectionFrame.balls)
  return balls_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::SSL_DetectionBall >*
SSL_DetectionFrame::mutable_balls() {
  // @@protoc_insertion_point(field_mutable_list:SSL_DetectionFrame.balls)
  return &balls_;
}
inline const ::SSL_DetectionBall& SSL_DetectionFrame::balls(int index) const {
  // @@protoc_insertion_point(field_get:SSL_DetectionFrame.balls)
  return balls_.Get(index);
}
inline ::SSL_DetectionBall* SSL_DetectionFrame::add_balls() {
  // @@protoc_insertion_point(field_add:SSL_DetectionFrame.balls)
  return balls_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::SSL_DetectionBall >&
SSL_DetectionFrame::balls() const {
  // @@protoc_insertion_point(field_list:SSL_DetectionFrame.balls)
  return balls_;
}

// repeated .SSL_DetectionRobot robots_yellow = 6;
inline int SSL_DetectionFrame::robots_yellow_size() const {
  return robots_yellow_.size();
}
inline void SSL_DetectionFrame::clear_robots_yellow() {
  robots_yellow_.Clear();
}
inline ::SSL_DetectionRobot* SSL_DetectionFrame::mutable_robots_yellow(int index) {
  // @@protoc_insertion_point(field_mutable:SSL_DetectionFrame.robots_yellow)
  return robots_yellow_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::SSL_DetectionRobot >*
SSL_DetectionFrame::mutable_robots_yellow() {
  // @@protoc_insertion_point(field_mutable_list:SSL_DetectionFrame.robots_yellow)
  return &robots_yellow_;
}
inline const ::SSL_DetectionRobot& SSL_DetectionFrame::robots_yellow(int index) const {
  // @@protoc_insertion_point(field_get:SSL_DetectionFrame.robots_yellow)
  return robots_yellow_.Get(index);
}
inline ::SSL_DetectionRobot* SSL_DetectionFrame::add_robots_yellow() {
  // @@protoc_insertion_point(field_add:SSL_DetectionFrame.robots_yellow)
  return robots_yellow_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::SSL_DetectionRobot >&
SSL_DetectionFrame::robots_yellow() const {
  // @@protoc_insertion_point(field_list:SSL_DetectionFrame.robots_yellow)
  return robots_yellow_;
}

// repeated .SSL_DetectionRobot robots_blue = 7;
inline int SSL_DetectionFrame::robots_blue_size() const {
  return robots_blue_.size();
}
inline void SSL_DetectionFrame::clear_robots_blue() {
  robots_blue_.Clear();
}
inline ::SSL_DetectionRobot* SSL_DetectionFrame::mutable_robots_blue(int index) {
  // @@protoc_insertion_point(field_mutable:SSL_DetectionFrame.robots_blue)
  return robots_blue_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::SSL_DetectionRobot >*
SSL_DetectionFrame::mutable_robots_blue() {
  // @@protoc_insertion_point(field_mutable_list:SSL_DetectionFrame.robots_blue)
  return &robots_blue_;
}
inline const ::SSL_DetectionRobot& SSL_DetectionFrame::robots_blue(int index) const {
  // @@protoc_insertion_point(field_get:SSL_DetectionFrame.robots_blue)
  return robots_blue_.Get(index);
}
inline ::SSL_DetectionRobot* SSL_DetectionFrame::add_robots_blue() {
  // @@protoc_insertion_point(field_add:SSL_DetectionFrame.robots_blue)
  return robots_blue_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::SSL_DetectionRobot >&
SSL_DetectionFrame::robots_blue() const {
  // @@protoc_insertion_point(field_list:SSL_DetectionFrame.robots_blue)
  return robots_blue_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)


// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_ssl_5fvision_5fdetection_2eproto
