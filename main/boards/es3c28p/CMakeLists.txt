# File: main/boards/es3c28p/CMakeLists.txt

set(board_sources
        "es3c28p_board.cc"
        "es3c28p_audio_codec.cc"
)

# Thêm đường dẫn "." và ".." (thư mục boards) và "../.." (thư mục main)
set(board_includes
        "."
        ".."
        "../.."
)

set(board_sources ${board_sources} PARENT_SCOPE)
set(board_includes ${board_includes} PARENT_SCOPE)