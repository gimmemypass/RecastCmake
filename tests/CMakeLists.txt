include(FetchContent)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

set(TEST_NAME recast_tests)

add_executable(${TEST_NAME} recast_tests.cpp)

target_link_libraries(
    ${TEST_NAME}
    GTest::gtest_main
    Recast
)

# target_include_directories(
    # ${TEST_NAME}
    # PUBLIC "../src/Shell/Include"
# )

include(GoogleTest)

gtest_discover_tests(${TEST_NAME})
