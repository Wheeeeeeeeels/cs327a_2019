# create an executable
ADD_EXECUTABLE (final-exam final-exam.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (final-exam ${CS327A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS327A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/final_exam)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_panda.urdf DESTINATION ${APP_RESOURCE_DIR})
