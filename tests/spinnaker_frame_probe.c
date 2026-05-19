#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "SpinnakerC.h"

#define ERROR_BUFFER_LEN 512

static const char *last_spinnaker_error(void) {
  static char buffer[ERROR_BUFFER_LEN];
  size_t len = sizeof(buffer);
  buffer[0] = '\0';
  spinErrorGetLastMessage(buffer, &len);
  return buffer;
}

static int report_error(const char *context, spinError err) {
  fprintf(stderr, "%s: %s [%d]\n", context, last_spinnaker_error(), (int)err);
  return err == SPINNAKER_ERR_SUCCESS ? 1 : (int)err;
}

static spinError set_enum_entry(spinNodeMapHandle node_map, const char *node_name, const char *entry_name,
                                int required) {
  spinError err = SPINNAKER_ERR_SUCCESS;
  spinNodeHandle node = NULL;
  spinNodeHandle entry = NULL;
  bool8_t readable = False;
  bool8_t writable = False;
  int64_t entry_value = 0;

  err = spinNodeMapGetNode(node_map, node_name, &node);
  if (err != SPINNAKER_ERR_SUCCESS || node == NULL) {
    if (required) {
      return err == SPINNAKER_ERR_SUCCESS ? SPINNAKER_ERR_ERROR : err;
    }
    return SPINNAKER_ERR_SUCCESS;
  }

  err = spinNodeIsReadable(node, &readable);
  if (err != SPINNAKER_ERR_SUCCESS) {
    return required ? err : SPINNAKER_ERR_SUCCESS;
  }
  err = spinNodeIsWritable(node, &writable);
  if (err != SPINNAKER_ERR_SUCCESS) {
    return required ? err : SPINNAKER_ERR_SUCCESS;
  }
  if (!readable || !writable) {
    return required ? SPINNAKER_ERR_ACCESS_DENIED : SPINNAKER_ERR_SUCCESS;
  }

  err = spinEnumerationGetEntryByName(node, entry_name, &entry);
  if (err != SPINNAKER_ERR_SUCCESS || entry == NULL) {
    return required ? err : SPINNAKER_ERR_SUCCESS;
  }

  err = spinEnumerationEntryGetIntValue(entry, &entry_value);
  if (err != SPINNAKER_ERR_SUCCESS) {
    return required ? err : SPINNAKER_ERR_SUCCESS;
  }

  err = spinEnumerationSetIntValue(node, entry_value);
  if (err != SPINNAKER_ERR_SUCCESS) {
    return required ? err : SPINNAKER_ERR_SUCCESS;
  }

  return SPINNAKER_ERR_SUCCESS;
}

static spinError set_linux_socket_stream_mode(spinCamera camera) {
  spinNodeMapHandle stream_node_map = NULL;
  spinError err = spinCameraGetTLStreamNodeMap(camera, &stream_node_map);
  if (err != SPINNAKER_ERR_SUCCESS) {
    return err;
  }
  return set_enum_entry(stream_node_map, "StreamMode", "Socket", 0);
}

static void usage(const char *program) { fprintf(stderr, "usage: %s --serial SERIAL [--timeout-ms MS]\n", program); }

int main(int argc, char **argv) {
  const char *serial = NULL;
  uint64_t timeout_ms = 5000;

  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--serial") == 0 && i + 1 < argc) {
      serial = argv[++i];
    } else if (strcmp(argv[i], "--timeout-ms") == 0 && i + 1 < argc) {
      timeout_ms = (uint64_t)strtoull(argv[++i], NULL, 10);
    } else {
      usage(argv[0]);
      return 2;
    }
  }

  if (serial == NULL || strlen(serial) == 0) {
    usage(argv[0]);
    return 2;
  }

  spinError err = SPINNAKER_ERR_SUCCESS;
  int exit_code = 1;
  spinSystem system = NULL;
  spinCameraList camera_list = NULL;
  spinCamera camera = NULL;
  spinImage image = NULL;
  int camera_initialized = 0;
  int acquisition_started = 0;

  err = spinSystemGetInstance(&system);
  if (err != SPINNAKER_ERR_SUCCESS) {
    exit_code = report_error("spinSystemGetInstance", err);
    goto cleanup;
  }

  err = spinCameraListCreateEmpty(&camera_list);
  if (err != SPINNAKER_ERR_SUCCESS) {
    exit_code = report_error("spinCameraListCreateEmpty", err);
    goto cleanup;
  }

  err = spinSystemGetCameras(system, camera_list);
  if (err != SPINNAKER_ERR_SUCCESS) {
    exit_code = report_error("spinSystemGetCameras", err);
    goto cleanup;
  }

  size_t camera_count = 0;
  err = spinCameraListGetSize(camera_list, &camera_count);
  if (err != SPINNAKER_ERR_SUCCESS) {
    exit_code = report_error("spinCameraListGetSize", err);
    goto cleanup;
  }

  err = spinCameraListGetBySerial(camera_list, serial, &camera);
  if (err != SPINNAKER_ERR_SUCCESS || camera == NULL) {
    fprintf(stderr, "camera serial %s not found or not available; cameras_detected=%zu; error=%s [%d]\n", serial,
            camera_count, last_spinnaker_error(), (int)err);
    exit_code = err == SPINNAKER_ERR_SUCCESS ? 1 : (int)err;
    goto cleanup;
  }

  err = spinCameraInit(camera);
  if (err != SPINNAKER_ERR_SUCCESS) {
    exit_code = report_error("spinCameraInit", err);
    goto cleanup;
  }
  camera_initialized = 1;

  err = set_linux_socket_stream_mode(camera);
  if (err != SPINNAKER_ERR_SUCCESS) {
    exit_code = report_error("set_linux_socket_stream_mode", err);
    goto cleanup;
  }

  spinNodeMapHandle node_map = NULL;
  err = spinCameraGetNodeMap(camera, &node_map);
  if (err != SPINNAKER_ERR_SUCCESS) {
    exit_code = report_error("spinCameraGetNodeMap", err);
    goto cleanup;
  }

  err = set_enum_entry(node_map, "AcquisitionMode", "Continuous", 1);
  if (err != SPINNAKER_ERR_SUCCESS) {
    exit_code = report_error("set AcquisitionMode=Continuous", err);
    goto cleanup;
  }

  err = spinCameraBeginAcquisition(camera);
  if (err != SPINNAKER_ERR_SUCCESS) {
    exit_code = report_error("spinCameraBeginAcquisition", err);
    goto cleanup;
  }
  acquisition_started = 1;

  err = spinCameraGetNextImageEx(camera, timeout_ms, &image);
  if (err != SPINNAKER_ERR_SUCCESS || image == NULL) {
    exit_code = report_error("spinCameraGetNextImageEx", err);
    goto cleanup;
  }

  bool8_t incomplete = False;
  err = spinImageIsIncomplete(image, &incomplete);
  if (err != SPINNAKER_ERR_SUCCESS) {
    exit_code = report_error("spinImageIsIncomplete", err);
    goto cleanup;
  }
  if (incomplete) {
    spinImageStatus status = SPINNAKER_IMAGE_STATUS_NO_ERROR;
    spinImageGetStatus(image, &status);
    fprintf(stderr, "image incomplete; status=%d\n", (int)status);
    exit_code = 1;
    goto cleanup;
  }

  size_t width = 0;
  size_t height = 0;
  size_t buffer_size = 0;
  void *data = NULL;
  err = spinImageGetWidth(image, &width);
  if (err != SPINNAKER_ERR_SUCCESS) {
    exit_code = report_error("spinImageGetWidth", err);
    goto cleanup;
  }
  err = spinImageGetHeight(image, &height);
  if (err != SPINNAKER_ERR_SUCCESS) {
    exit_code = report_error("spinImageGetHeight", err);
    goto cleanup;
  }
  err = spinImageGetBufferSize(image, &buffer_size);
  if (err != SPINNAKER_ERR_SUCCESS) {
    exit_code = report_error("spinImageGetBufferSize", err);
    goto cleanup;
  }
  err = spinImageGetData(image, &data);
  if (err != SPINNAKER_ERR_SUCCESS) {
    exit_code = report_error("spinImageGetData", err);
    goto cleanup;
  }

  if (width == 0 || height == 0 || buffer_size == 0 || data == NULL) {
    fprintf(stderr, "invalid image payload; serial=%s width=%zu height=%zu bytes=%zu data=%p\n", serial, width, height,
            buffer_size, data);
    exit_code = 1;
    goto cleanup;
  }

  const unsigned char *bytes = (const unsigned char *)data;
  size_t sample_len = buffer_size < 4096 ? buffer_size : 4096;
  int nonzero_sample = 0;
  for (size_t i = 0; i < sample_len; ++i) {
    if (bytes[i] != 0) {
      nonzero_sample = 1;
      break;
    }
  }

  if (!nonzero_sample) {
    fprintf(stderr, "image payload sample is all zeros; serial=%s bytes=%zu\n", serial, buffer_size);
    exit_code = 1;
    goto cleanup;
  }

  printf("frame_ok serial=%s width=%zu height=%zu bytes=%zu cameras_detected=%zu\n", serial, width, height, buffer_size,
         camera_count);
  exit_code = 0;

cleanup:
  if (image != NULL) {
    spinImageRelease(image);
  }
  if (acquisition_started) {
    spinCameraEndAcquisition(camera);
  }
  if (camera_initialized) {
    spinCameraDeInit(camera);
  }
  if (camera != NULL) {
    spinCameraRelease(camera);
  }
  if (camera_list != NULL) {
    spinCameraListClear(camera_list);
    spinCameraListDestroy(camera_list);
  }
  if (system != NULL) {
    spinSystemReleaseInstance(system);
  }

  return exit_code;
}
