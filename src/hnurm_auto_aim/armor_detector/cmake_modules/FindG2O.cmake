# Locate the g2o libraries
# A general framework for graph optimization.
#
# This module defines
# G2O_FOUND, if false, do not try to link against g2o
# G2O_LIBRARIES, path to the libg2o
# G2O_INCLUDE_DIR, where to find the g2o header files
#
# Niko Suenderhauf <niko@etit.tu-chemnitz.de>
# Adapted by Felix Endres <endres@informatik.uni-freiburg.de>

IF(UNIX)

  #IF(G2O_INCLUDE_DIR AND G2O_LIBRARIES)
    # in cache already
    #  SET(G2O_FIND_QUIETLY TRUE)
    #ENDIF(G2O_INCLUDE_DIR AND G2O_LIBRARIES)

  MESSAGE(STATUS "Searching for g2o ...")
  
  # First try to use g2o's modern CMake config if available
  find_package(g2o QUIET NO_MODULE)
  if(g2o_FOUND)
    MESSAGE(STATUS "Found g2o using modern CMake config")
    set(G2O_FOUND TRUE)
    # Set compatibility variables
    if(TARGET g2o::core)
      get_target_property(G2O_INCLUDE_DIR g2o::core INTERFACE_INCLUDE_DIRECTORIES)
      # For compatibility with old code, also set G2O_INCLUDE_DIRS
      set(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
    # Set libraries for compatibility
    set(G2O_LIBRARIES 
      g2o::core
      g2o::stuff
      g2o::types_slam3d
      g2o::types_sba
      g2o::solver_dense
    )
      MESSAGE(STATUS "G2O_INCLUDE_DIR: ${G2O_INCLUDE_DIR}")
      MESSAGE(STATUS "G2O_LIBRARIES: ${G2O_LIBRARIES}")
    endif()
  else()
    # Fall back to traditional find methods
    MESSAGE(STATUS "Modern g2o config not found, using traditional search")
    
    # Try to find include directory
    FIND_PATH(G2O_INCLUDE_DIR
      NAMES core math_groups types
      PATHS /usr/local /usr /opt/local
      PATH_SUFFIXES include/g2o include)

    IF (G2O_INCLUDE_DIR)
      MESSAGE(STATUS "Found g2o headers in: ${G2O_INCLUDE_DIR}")
      # For compatibility with old code
      set(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
    ELSE()
      MESSAGE(STATUS "Could not find g2o headers")
    ENDIF ()

    # Try to find libraries - use more search paths and names
    FIND_LIBRARY(G2O_CORE_LIB             
      NAMES g2o_core
      PATHS /usr/local/lib /usr/lib /opt/local/lib ${CMAKE_PREFIX_PATH}/lib
      NO_DEFAULT_PATH)
      
    FIND_LIBRARY(G2O_STUFF_LIB            
      NAMES g2o_stuff
      PATHS /usr/local/lib /usr/lib /opt/local/lib ${CMAKE_PREFIX_PATH}/lib
      NO_DEFAULT_PATH)
      
    FIND_LIBRARY(G2O_TYPES_SBA_LIB
      NAMES g2o_types_sba
      PATHS /usr/local/lib /usr/lib /opt/local/lib ${CMAKE_PREFIX_PATH}/lib
      NO_DEFAULT_PATH)
      
    FIND_LIBRARY(G2O_TYPES_SLAM3D_LIB     
      NAMES g2o_types_slam3d
      PATHS /usr/local/lib /usr/lib /opt/local/lib ${CMAKE_PREFIX_PATH}/lib
      NO_DEFAULT_PATH)
      
    FIND_LIBRARY(G2O_SOLVER_DENSE_LIB
      NAMES g2o_solver_dense
      PATHS /usr/local/lib /usr/lib /opt/local/lib ${CMAKE_PREFIX_PATH}/lib
      NO_DEFAULT_PATH)

    MESSAGE(STATUS "G2O_CORE_LIB: ${G2O_CORE_LIB}")
    MESSAGE(STATUS "G2O_STUFF_LIB: ${G2O_STUFF_LIB}")
    MESSAGE(STATUS "G2O_TYPES_SBA_LIB: ${G2O_TYPES_SBA_LIB}")
    MESSAGE(STATUS "G2O_TYPES_SLAM3D_LIB: ${G2O_TYPES_SLAM3D_LIB}")
    MESSAGE(STATUS "G2O_SOLVER_DENSE_LIB: ${G2O_SOLVER_DENSE_LIB}")

    # Set the libraries variable
    SET(G2O_LIBRARIES 
      ${G2O_CORE_LIB}
      ${G2O_STUFF_LIB}
      ${G2O_TYPES_SBA_LIB}
      ${G2O_TYPES_SLAM3D_LIB}
      ${G2O_SOLVER_DENSE_LIB}
    )

    # Check if we found everything
    IF(G2O_LIBRARIES AND G2O_INCLUDE_DIR)
      SET(G2O_FOUND "YES")
      IF(NOT G2O_FIND_QUIETLY)
        MESSAGE(STATUS "Found libg2o: ${G2O_LIBRARIES}")
      ENDIF()
    ELSE()
      IF(NOT G2O_LIBRARIES)
        MESSAGE(STATUS "Could not find g2o libraries")
        IF(G2O_FIND_REQUIRED)
          message(FATAL_ERROR "Could not find libg2o!")
        ENDIF()
      ENDIF()

      IF(NOT G2O_INCLUDE_DIR)
        MESSAGE(STATUS "Could not find g2o include directory")
        IF(G2O_FIND_REQUIRED)
          message(FATAL_ERROR "Could not find g2o include directory!")
        ENDIF()
      ENDIF()
    ENDIF()
  endif()  # Close if(g2o_FOUND) block
  
ENDIF(UNIX)
