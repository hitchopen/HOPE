// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from audio_msgs:srv/AbandonAudioFocus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "audio_msgs/srv/abandon_audio_focus.h"


#ifndef AUDIO_MSGS__SRV__DETAIL__ABANDON_AUDIO_FOCUS__FUNCTIONS_H_
#define AUDIO_MSGS__SRV__DETAIL__ABANDON_AUDIO_FOCUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "audio_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "audio_msgs/srv/detail/abandon_audio_focus__struct.h"

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_type_hash_t *
audio_msgs__srv__AbandonAudioFocus__get_type_hash(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_runtime_c__type_description__TypeDescription *
audio_msgs__srv__AbandonAudioFocus__get_type_description(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_runtime_c__type_description__TypeSource *
audio_msgs__srv__AbandonAudioFocus__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_msgs__srv__AbandonAudioFocus__get_type_description_sources(
  const rosidl_service_type_support_t * type_support);

/// Initialize srv/AbandonAudioFocus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * audio_msgs__srv__AbandonAudioFocus_Request
 * )) before or use
 * audio_msgs__srv__AbandonAudioFocus_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Request__init(audio_msgs__srv__AbandonAudioFocus_Request * msg);

/// Finalize srv/AbandonAudioFocus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
void
audio_msgs__srv__AbandonAudioFocus_Request__fini(audio_msgs__srv__AbandonAudioFocus_Request * msg);

/// Create srv/AbandonAudioFocus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * audio_msgs__srv__AbandonAudioFocus_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
audio_msgs__srv__AbandonAudioFocus_Request *
audio_msgs__srv__AbandonAudioFocus_Request__create(void);

/// Destroy srv/AbandonAudioFocus message.
/**
 * It calls
 * audio_msgs__srv__AbandonAudioFocus_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
void
audio_msgs__srv__AbandonAudioFocus_Request__destroy(audio_msgs__srv__AbandonAudioFocus_Request * msg);

/// Check for srv/AbandonAudioFocus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Request__are_equal(const audio_msgs__srv__AbandonAudioFocus_Request * lhs, const audio_msgs__srv__AbandonAudioFocus_Request * rhs);

/// Copy a srv/AbandonAudioFocus message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Request__copy(
  const audio_msgs__srv__AbandonAudioFocus_Request * input,
  audio_msgs__srv__AbandonAudioFocus_Request * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_type_hash_t *
audio_msgs__srv__AbandonAudioFocus_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_runtime_c__type_description__TypeDescription *
audio_msgs__srv__AbandonAudioFocus_Request__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_runtime_c__type_description__TypeSource *
audio_msgs__srv__AbandonAudioFocus_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_msgs__srv__AbandonAudioFocus_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/AbandonAudioFocus messages.
/**
 * It allocates the memory for the number of elements and calls
 * audio_msgs__srv__AbandonAudioFocus_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Request__Sequence__init(audio_msgs__srv__AbandonAudioFocus_Request__Sequence * array, size_t size);

/// Finalize array of srv/AbandonAudioFocus messages.
/**
 * It calls
 * audio_msgs__srv__AbandonAudioFocus_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
void
audio_msgs__srv__AbandonAudioFocus_Request__Sequence__fini(audio_msgs__srv__AbandonAudioFocus_Request__Sequence * array);

/// Create array of srv/AbandonAudioFocus messages.
/**
 * It allocates the memory for the array and calls
 * audio_msgs__srv__AbandonAudioFocus_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
audio_msgs__srv__AbandonAudioFocus_Request__Sequence *
audio_msgs__srv__AbandonAudioFocus_Request__Sequence__create(size_t size);

/// Destroy array of srv/AbandonAudioFocus messages.
/**
 * It calls
 * audio_msgs__srv__AbandonAudioFocus_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
void
audio_msgs__srv__AbandonAudioFocus_Request__Sequence__destroy(audio_msgs__srv__AbandonAudioFocus_Request__Sequence * array);

/// Check for srv/AbandonAudioFocus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Request__Sequence__are_equal(const audio_msgs__srv__AbandonAudioFocus_Request__Sequence * lhs, const audio_msgs__srv__AbandonAudioFocus_Request__Sequence * rhs);

/// Copy an array of srv/AbandonAudioFocus messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Request__Sequence__copy(
  const audio_msgs__srv__AbandonAudioFocus_Request__Sequence * input,
  audio_msgs__srv__AbandonAudioFocus_Request__Sequence * output);

/// Initialize srv/AbandonAudioFocus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * audio_msgs__srv__AbandonAudioFocus_Response
 * )) before or use
 * audio_msgs__srv__AbandonAudioFocus_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Response__init(audio_msgs__srv__AbandonAudioFocus_Response * msg);

/// Finalize srv/AbandonAudioFocus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
void
audio_msgs__srv__AbandonAudioFocus_Response__fini(audio_msgs__srv__AbandonAudioFocus_Response * msg);

/// Create srv/AbandonAudioFocus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * audio_msgs__srv__AbandonAudioFocus_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
audio_msgs__srv__AbandonAudioFocus_Response *
audio_msgs__srv__AbandonAudioFocus_Response__create(void);

/// Destroy srv/AbandonAudioFocus message.
/**
 * It calls
 * audio_msgs__srv__AbandonAudioFocus_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
void
audio_msgs__srv__AbandonAudioFocus_Response__destroy(audio_msgs__srv__AbandonAudioFocus_Response * msg);

/// Check for srv/AbandonAudioFocus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Response__are_equal(const audio_msgs__srv__AbandonAudioFocus_Response * lhs, const audio_msgs__srv__AbandonAudioFocus_Response * rhs);

/// Copy a srv/AbandonAudioFocus message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Response__copy(
  const audio_msgs__srv__AbandonAudioFocus_Response * input,
  audio_msgs__srv__AbandonAudioFocus_Response * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_type_hash_t *
audio_msgs__srv__AbandonAudioFocus_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_runtime_c__type_description__TypeDescription *
audio_msgs__srv__AbandonAudioFocus_Response__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_runtime_c__type_description__TypeSource *
audio_msgs__srv__AbandonAudioFocus_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_msgs__srv__AbandonAudioFocus_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/AbandonAudioFocus messages.
/**
 * It allocates the memory for the number of elements and calls
 * audio_msgs__srv__AbandonAudioFocus_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Response__Sequence__init(audio_msgs__srv__AbandonAudioFocus_Response__Sequence * array, size_t size);

/// Finalize array of srv/AbandonAudioFocus messages.
/**
 * It calls
 * audio_msgs__srv__AbandonAudioFocus_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
void
audio_msgs__srv__AbandonAudioFocus_Response__Sequence__fini(audio_msgs__srv__AbandonAudioFocus_Response__Sequence * array);

/// Create array of srv/AbandonAudioFocus messages.
/**
 * It allocates the memory for the array and calls
 * audio_msgs__srv__AbandonAudioFocus_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
audio_msgs__srv__AbandonAudioFocus_Response__Sequence *
audio_msgs__srv__AbandonAudioFocus_Response__Sequence__create(size_t size);

/// Destroy array of srv/AbandonAudioFocus messages.
/**
 * It calls
 * audio_msgs__srv__AbandonAudioFocus_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
void
audio_msgs__srv__AbandonAudioFocus_Response__Sequence__destroy(audio_msgs__srv__AbandonAudioFocus_Response__Sequence * array);

/// Check for srv/AbandonAudioFocus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Response__Sequence__are_equal(const audio_msgs__srv__AbandonAudioFocus_Response__Sequence * lhs, const audio_msgs__srv__AbandonAudioFocus_Response__Sequence * rhs);

/// Copy an array of srv/AbandonAudioFocus messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Response__Sequence__copy(
  const audio_msgs__srv__AbandonAudioFocus_Response__Sequence * input,
  audio_msgs__srv__AbandonAudioFocus_Response__Sequence * output);

/// Initialize srv/AbandonAudioFocus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * audio_msgs__srv__AbandonAudioFocus_Event
 * )) before or use
 * audio_msgs__srv__AbandonAudioFocus_Event__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Event__init(audio_msgs__srv__AbandonAudioFocus_Event * msg);

/// Finalize srv/AbandonAudioFocus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
void
audio_msgs__srv__AbandonAudioFocus_Event__fini(audio_msgs__srv__AbandonAudioFocus_Event * msg);

/// Create srv/AbandonAudioFocus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * audio_msgs__srv__AbandonAudioFocus_Event__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
audio_msgs__srv__AbandonAudioFocus_Event *
audio_msgs__srv__AbandonAudioFocus_Event__create(void);

/// Destroy srv/AbandonAudioFocus message.
/**
 * It calls
 * audio_msgs__srv__AbandonAudioFocus_Event__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
void
audio_msgs__srv__AbandonAudioFocus_Event__destroy(audio_msgs__srv__AbandonAudioFocus_Event * msg);

/// Check for srv/AbandonAudioFocus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Event__are_equal(const audio_msgs__srv__AbandonAudioFocus_Event * lhs, const audio_msgs__srv__AbandonAudioFocus_Event * rhs);

/// Copy a srv/AbandonAudioFocus message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Event__copy(
  const audio_msgs__srv__AbandonAudioFocus_Event * input,
  audio_msgs__srv__AbandonAudioFocus_Event * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_type_hash_t *
audio_msgs__srv__AbandonAudioFocus_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_runtime_c__type_description__TypeDescription *
audio_msgs__srv__AbandonAudioFocus_Event__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_runtime_c__type_description__TypeSource *
audio_msgs__srv__AbandonAudioFocus_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
audio_msgs__srv__AbandonAudioFocus_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/AbandonAudioFocus messages.
/**
 * It allocates the memory for the number of elements and calls
 * audio_msgs__srv__AbandonAudioFocus_Event__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Event__Sequence__init(audio_msgs__srv__AbandonAudioFocus_Event__Sequence * array, size_t size);

/// Finalize array of srv/AbandonAudioFocus messages.
/**
 * It calls
 * audio_msgs__srv__AbandonAudioFocus_Event__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
void
audio_msgs__srv__AbandonAudioFocus_Event__Sequence__fini(audio_msgs__srv__AbandonAudioFocus_Event__Sequence * array);

/// Create array of srv/AbandonAudioFocus messages.
/**
 * It allocates the memory for the array and calls
 * audio_msgs__srv__AbandonAudioFocus_Event__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
audio_msgs__srv__AbandonAudioFocus_Event__Sequence *
audio_msgs__srv__AbandonAudioFocus_Event__Sequence__create(size_t size);

/// Destroy array of srv/AbandonAudioFocus messages.
/**
 * It calls
 * audio_msgs__srv__AbandonAudioFocus_Event__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
void
audio_msgs__srv__AbandonAudioFocus_Event__Sequence__destroy(audio_msgs__srv__AbandonAudioFocus_Event__Sequence * array);

/// Check for srv/AbandonAudioFocus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Event__Sequence__are_equal(const audio_msgs__srv__AbandonAudioFocus_Event__Sequence * lhs, const audio_msgs__srv__AbandonAudioFocus_Event__Sequence * rhs);

/// Copy an array of srv/AbandonAudioFocus messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_audio_msgs
bool
audio_msgs__srv__AbandonAudioFocus_Event__Sequence__copy(
  const audio_msgs__srv__AbandonAudioFocus_Event__Sequence * input,
  audio_msgs__srv__AbandonAudioFocus_Event__Sequence * output);
#ifdef __cplusplus
}
#endif

#endif  // AUDIO_MSGS__SRV__DETAIL__ABANDON_AUDIO_FOCUS__FUNCTIONS_H_
