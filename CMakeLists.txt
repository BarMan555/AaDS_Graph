cmake_minimum_required(VERSION 3.11)

project(GraphApp CXX)

set(CMAKE_CXX_STANDARD 23)

# ����������� ��������� ��� ����������� Visual Studio
if(MSVC)
    # ���������� ����������� �������� ��� ������������ ���������
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS true)

    # �������� ������� ������� ��������������
    if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    endif()
endif()

# ���������� googletest
include(FetchContent)
FetchContent_Declare(googletest URL https://github.com/google/googletest/archive/6b63c98ac43efc992122f1da12aaf0a0e0658206.zip)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ��������� ����������
add_library(graph_lib SHARED "include/graph.h" "src/graph.cpp")
target_include_directories(graph_lib PUBLIC "include")

# ��������� �����
add_executable(graph_test "test/graph_test.cpp")
target_link_libraries(graph_test graph_lib GTest::gtest_main)

# ��������� ���������� ����������
add_executable(app "app/main.cpp")
target_link_libraries(app graph_lib)