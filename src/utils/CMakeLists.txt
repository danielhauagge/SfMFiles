ADD_EXECUTABLE(pmvs_info pmvs_info.cpp)
TARGET_LINK_LIBRARIES(pmvs_info SfMFiles)

ADD_EXECUTABLE(bundler_info bundler_info.cpp)
TARGET_LINK_LIBRARIES(bundler_info SfMFiles)

ADD_EXECUTABLE(pmvs_merge pmvs_merge.cpp)
TARGET_LINK_LIBRARIES(pmvs_merge SfMFiles) 

ADD_EXECUTABLE(bundler_merge bundler_merge.cpp)
TARGET_LINK_LIBRARIES(bundler_merge SfMFiles)

ADD_EXECUTABLE(pmvs_filter pmvs_filter.cpp)
TARGET_LINK_LIBRARIES(pmvs_filter SfMFiles)

ADD_EXECUTABLE(bundler_filter bundler_filter.cpp)
TARGET_LINK_LIBRARIES(bundler_filter SfMFiles)

ADD_EXECUTABLE(pmvs2ply pmvs2ply.cpp)
TARGET_LINK_LIBRARIES(pmvs2ply SfMFiles)

ADD_EXECUTABLE(bundler2ply bundler2ply.cpp)
TARGET_LINK_LIBRARIES(bundler2ply SfMFiles)

ADD_EXECUTABLE(pmvs2bundler pmvs2bundler.cpp)
TARGET_LINK_LIBRARIES(pmvs2bundler SfMFiles)

ADD_EXECUTABLE(bundler_rmdups bundler_rmdups.cpp)
TARGET_LINK_LIBRARIES(bundler_rmdups SfMFiles)

ADD_EXECUTABLE(bundler_transform bundler_transform.cpp)
TARGET_LINK_LIBRARIES(bundler_transform SfMFiles)

ADD_EXECUTABLE(pmvs_transform pmvs_transform.cpp)
TARGET_LINK_LIBRARIES(pmvs_transform SfMFiles)

ADD_EXECUTABLE(bundler_focal2list bundler_focal2list.cpp)
TARGET_LINK_LIBRARIES(bundler_focal2list SfMFiles)

ADD_EXECUTABLE(pmvs_check_remapped pmvs_check_remapped.cpp)
TARGET_LINK_LIBRARIES(pmvs_check_remapped SfMFiles)

INSTALL_PROGRAMS(/bin 
                 FILES 

                 bundler_merge 
                 bundler_info  
                 bundler_transform  
                 bundler_focal2list
                 bundler_rmdups
                 bundler_filter 
                 bundler2ply 

                 pmvs_merge 
                 pmvs_info 
                 pmvs_filter 
                 pmvs_transform
                 pmvs2ply 
                 pmvs2bundler
                 pmvs_check_remapped) 

