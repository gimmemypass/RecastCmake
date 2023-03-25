file(GLOB_RECURSE sources_Shell			    Shell/*.cpp						Shell/*.h)
file(GLOB_RECURSE sources_Recast			Recast/Source/*.cpp				Recast/Include/*.h)
file(GLOB_RECURSE sources_Detour			Detour/Source/*.cpp				Detour/Include/*.h)
file(GLOB_RECURSE sources_DetourCrowd		DetourCrowd/Source/*.cpp		DetourCrowd/Include/*.h)
file(GLOB_RECURSE sources_DetourTileCache   DetourTileCache/Source/*.cpp	DetourTileCache/Include/*.h)
file(GLOB_RECURSE sources_Debug             Debug/Source/*.cpp	            Debug/Include/*.h)

set(EPS_SRCS
  ${EPS_SRCS}
  ${sources_Shell}
  ${sources_Recast}
  ${sources_Detour}
  ${sources_DetourCrowd}
  ${sources_DetourTileCache}
  ${sources_Debug}
)



add_library(Recast SHARED ${EPS_SRCS})

target_include_directories(
    Recast
    PUBLIC Shell
    PUBLIC Recast/Include
    PUBLIC Recast/Source
    PUBLIC Detour/Include
    PUBLIC Detour/Source
    PUBLIC DetourCrowd/Include
    PUBLIC DetourCrowd/Source
    PUBLIC DetourTileCache/Include
    PUBLIC DetourTileCache/Source
    PUBLIC Debug/Include
    PUBLIC Debug/Source
    ${CMAKE_SOURCE_DIR}/dep/fastlz
)

if (BUILD_32_BITS AND NOT WIN32)
    set(COMPILE_FLAGS "${COMPILE_FLAGS} -m32")
    set(LINK_FLAGS "${LINK_FLAGS} -m32")
elseif (NOT WIN32)
    set(COMPILE_FLAGS "${COMPILE_FLAGS} -m64")
    set(LINK_FLAGS "${LINK_FLAGS} -m64")
endif()

if (NOT WIN32 OR BUILD_32_BITS)
    set_target_properties(Recast PROPERTIES COMPILE_FLAGS "${COMPILE_FLAGS}" LINK_FLAGS "${LINK_FLAGS}")
endif()

target_link_libraries(
    Recast
    FastLZ
)