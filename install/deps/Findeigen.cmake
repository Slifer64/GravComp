# Define some colors for ouput
if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()


# =============================
# Search for Eigen locally
# =============================
find_path( EIGEN_INCLUDE_DIR
  name signature_of_eigen3_matrix_library
  NO_DEFAULT_PATH
  PATHS ${CMAKE_BINARY_DIR}/../install/deps/eigen3
)

# set( EIGEN_INCLUDE_DIR ${CMAKE_BINARY_DIR}/../install/deps/eigen3 )

# =============================
# Check if Eigen was found locally
# =============================

set( EIGEN_FOUND false )

if(EIGEN_INCLUDE_DIR)
  message(STATUS "${Blue}Using local installation of EIGEN.${ColourReset}")
  set( EIGEN_FOUND true )
  set( EIGEN_LIBRARY "")
endif()


# =============================
# If Eigen was not found locally, search for global installation
# =============================
if(NOT EIGEN_FOUND)

  find_path( EIGEN_INCLUDE_DIR
    NAME signature_of_eigen3_matrix_library
    PATHS /usr/include/eigen3
  )

  if(EIGEN_INCLUDE_DIR)
    message(WARNING "${Yellow}Using global installation of EIGEN.${ColourReset}")
    set( EIGEN_FOUND true )
    set( EIGEN_INCLUDE_DIR /usr/include )
    set( EIGEN_LIBRARY "")
  endif()

endif()



if(EIGEN_FOUND)
  MESSAGE(STATUS
    "${Green}*** Found EIGEN ***\n"
    "${Blue}Include dir:${ColourReset}${EIGEN_INCLUDE_DIR}\n"
    "${Blue}Libaries:${ColourReset}${EIGEN_LIBRARY}"
  )
else()
  if(EIGEN_FIND_REQUIRED)
    message(FATAL_ERROR "${Red}Could not find EIGEN...${ColourReset}")
  else()
    message(WARNING "${Yellow}Could not find EIGEN...${ColourReset}")
  endif()
endif()
