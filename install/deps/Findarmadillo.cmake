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

find_package(LAPACK REQUIRED)

# =============================
# Search for armadillo locally
# =============================

# define arma version
# set( ARMA_VERSION "9.860.2")  # reduntant

# clear previous values
unset( ARMADILLO_INCLUDE_DIR CACHE)
unset( ARMADILLO_LIBRARY CACHE )

find_path( ARMADILLO_INCLUDE_DIR armadillo
  NO_DEFAULT_PATH
  PATHS ${CMAKE_SOURCE_DIR}/../install/deps/armadillo/include
)

find_library( ARMADILLO_LIBRARY
  NAME armadillo
  NO_DEFAULT_PATH
  PATHS ${CMAKE_SOURCE_DIR}/../install/deps/armadillo
)

# =============================
# Check if armadillo was found locally
# =============================
set( ARMADILLO_FOUND false )

if(ARMADILLO_INCLUDE_DIR AND ARMADILLO_LIBRARY)
  message(STATUS "${Blue}Using local installation of ARMADILLO.${ColourReset}")
  set( ARMADILLO_FOUND true )
  set( ARMADILLO_LIBRARY
       ${ARMADILLO_LIBRARY}
       ${LAPACK_LIBRARIES})
endif()


# =============================
# If armadillo was not found locally, search for global installation
# =============================
if(NOT ARMADILLO_FOUND)

  find_path( ARMADILLO_INCLUDE_DIR
    NAME armadillo
    PATHS /usr/include
  )

  find_library( ARMADILLO_LIBRARY
    NAME armadillo
    PATHS /usr/lib/x86_64-linux-gnu
  )

  if(ARMADILLO_INCLUDE_DIR AND ARMADILLO_LIBRARY)
    message(WARNING "${Yellow}Using global installation of ARMADILLO.${ColourReset}")
    set( ARMADILLO_FOUND true )
    set( ARMADILLO_INCLUDE_DIR /usr/include )
    set( ARMADILLO_LIBRARY
         ${ARMADILLO_LIBRARY}
         ${LAPACK_LIBRARIES})
  endif()

endif()


if(ARMADILLO_FOUND)
  MESSAGE(STATUS
    "${Green}*** Found ARMADILLO ***\n"
    "${Blue}Include dir:${ColourReset}${ARMADILLO_INCLUDE_DIR}\n"
    "${Blue}Libaries:${ColourReset}${ARMADILLO_LIBRARY}"
  )
else()
  if(ARMADILLO_FIND_REQUIRED)
    message(FATAL_ERROR "${Red}Could not find ARMADILLO...${ColourReset}")
  else()
    message(WARNING "${Yellow}Could not find ARMADILLO...${ColourReset}")
  endif()
endif()
