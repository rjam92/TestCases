file(GLOB_RECURSE FSM_SOURCES "src/*.*")


add_library(FSM "${FSM_SOURCES}")
target_include_directories(FSM PUBLIC include)

