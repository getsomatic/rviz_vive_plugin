if (NOT OPENVR_DIR)
  message(FATAL_ERROR "OPENVR_DIR variable is not set")
endif ()

set(OpenVR_LIBRARIES ${OPENVR_DIR}/lib/linux64/libopenvr_api.so)
set(OpenVR_INCLUDE_DIRS ${OPENVR_DIR}/headers)