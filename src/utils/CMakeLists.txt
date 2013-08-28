ADD_EXECUTABLE(bundler_file_converter bundler_file_converter.cpp)
TARGET_LINK_LIBRARIES(bundler_file_converter SfMFiles)

ADD_EXECUTABLE(pmvs_info pmvs_info.cpp)
TARGET_LINK_LIBRARIES(pmvs_info SfMFiles)

ADD_EXECUTABLE(pmvs_merge pmvs_merge.cpp)
TARGET_LINK_LIBRARIES(pmvs_merge SfMFiles) 

ADD_EXECUTABLE(pmvs_filter pmvs_filter.cpp)
TARGET_LINK_LIBRARIES(pmvs_filter SfMFiles)

ADD_EXECUTABLE(bundler_merge bundler_merge.cpp)
TARGET_LINK_LIBRARIES(bundler_merge SfMFiles)

ADD_EXECUTABLE(bundler_info bundler_info.cpp)
TARGET_LINK_LIBRARIES(bundler_info SfMFiles)

ADD_EXECUTABLE(bundler_rmdups bundler_rmdups.cpp)
TARGET_LINK_LIBRARIES(bundler_rmdups SfMFiles)

ADD_EXECUTABLE(pmvs2ply pmvs2ply.cpp)
TARGET_LINK_LIBRARIES(pmvs2ply SfMFiles)

ADD_EXECUTABLE(pmvs2bundler pmvs2bundler.cpp)
TARGET_LINK_LIBRARIES(pmvs2bundler SfMFiles)

