if(APPLE)
    message("-- HOME = $ENV{HOME}")
    file(GLOB dirs $ENV{HOME}/ILOG/CPLEX_Studio128 $ENV{HOME}/Applications/CPLEX_Studio128)
    foreach(d in ${dirs})
	    string(REGEX MATCH "Studio[0-9]+" CPLEX_VERSION "${d}")
    endforeach(d)
    string(SUBSTRING ${CPLEX_VERSION} 6 -1 CPLEX_VERSION)
endif()

message("-- Cplex version ${CPLEX_VERSION}")

if(APPLE)
    # string(CONCAT CPLEX_DIR $ENV{HOME}/Applications/CPLEX_Studio;${CPLEX_VERSION};/cplex)
    # string(CONCAT CONCERT_DIR $ENV{HOME}/Applications/CPLEX_Studio;${CPLEX_VERSION};/concert)
    string(CONCAT CPLEX_DIR $ENV{HOME}/ILOG/CPLEX_Studio128/cplex)
    string(CONCAT CONCERT_DIR $ENV{HOME}/ILOG/CPLEX_Studio128/concert)
elseif(UNIX)
    string(CONCAT CPLEX_DIR /home/diode/cplex/cplex)
    string(CONCAT CONCERT_DIR /home/diode/cplex/concert)
endif()

message("-- Looking for Cplex in ${CPLEX_DIR}")

if(APPLE)
    find_path(CPLEX_INCLUDE_DIR ilocplex.h HINTS "${CPLEX_DIR}/include/ilcplex")
    find_path(CONCERT_INCLUDE_DIR iloalg.h HINTS "${CONCERT_DIR}/include/ilconcert")
    find_library(CPLEX_CPP_LIBRARY libcplex.a HINTS "${CPLEX_DIR}/lib/x86-64_osx/static_pic")
    find_library(CONCERT_CPP_LIBRARY libconcert.a HINTS "${CONCERT_DIR}/lib/x86-64_osx/static_pic")
    find_library(CPLEX_ILO_CPP_LIBRARY libilocplex.a HINTS "${CPLEX_DIR}/lib/x86-64_osx/static_pic")
elseif(UNIX)
    find_path(CPLEX_INCLUDE_DIR ilocplex.h HINTS "${CPLEX_DIR}/include/ilcplex")
    find_path(CONCERT_INCLUDE_DIR iloalg.h HINTS "${CONCERT_DIR}/include/ilconcert")
    
    message("Found for Cplex Link directory")
    
    find_library(CPLEX_CPP_LIBRARY libcplex.a x HINTS "${CPLEX_DIR}/lib/x86-64_linux/static_pic")
    find_library(CONCERT_CPP_LIBRARY libconcert.a libcplexdistmip.a HINTS "${CONCERT_DIR}/lib/x86-64_linux/static_pic")
    find_library(CPLEX_ILO_CPP_LIBRARY libilocplex.a HINTS "${CPLEX_DIR}/lib/x86-64_linux/static_pic")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CPLEX DEFAULT_MSG CPLEX_CPP_LIBRARY  CPLEX_ILO_CPP_LIBRARY CONCERT_CPP_LIBRARY  CPLEX_INCLUDE_DIR CONCERT_INCLUDE_DIR)

if(CPLEX_FOUND)
    set(CPLEX_INCLUDE_DIRS ${CPLEX_INCLUDE_DIR}/.. ${CONCERT_INCLUDE_DIR}/..)
    set(CPLEX_LIBRARIES ${CPLEX_ILO_CPP_LIBRARY} ${CPLEX_CPP_LIBRARY}  ${CONCERT_CPP_LIBRARY})
endif(CPLEX_FOUND)

mark_as_advanced(CPLEX_LIBRARY CPLEX_CPP_LIBRARY CPLEX_INCLUDE_DIR)
