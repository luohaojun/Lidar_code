cmake_minimum_required(VERSION 2.6)
project(pcl_use)
add_definitions(-std=c++14)

add_executable(pcl_use pcl_use.cpp)
add_executable(nearestKSearch nearestKSearch.cpp)
add_executable(ICP_test ICP_test.cpp)
add_executable(4PCS_test 4PCS_test.cpp)
add_executable(Registration Registration.cpp)
add_executable(SelectPC SelectPC.cpp)
add_executable(ExtractPC ExtractPC.cpp)
add_executable(PC_splicing PC_splicing.cpp)
add_executable (PCL_cluster PCL_cluster.cpp)
add_executable (pcl_viewer pcl_viewer.cpp)
add_executable (SegRansac SegRansac.cpp)
add_executable (cluster_extraction cluster_extraction.cpp)
add_executable (Caculate_R Caculate_R.cpp)
add_executable (ope_pcl ope_pcl.cpp)
add_executable (ply2pcd ply2pcd.cpp)
add_executable (pcl_correspondence pcl_correspondence.cpp)


find_package(PCL 1.10 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries (pcl_use ${PCL_LIBRARIES})
target_link_libraries (ICP_test ${PCL_LIBRARIES})
target_link_libraries (4PCS_test ${PCL_LIBRARIES})
target_link_libraries (PCL_cluster ${PCL_LIBRARIES})
target_link_libraries (pcl_viewer ${PCL_LIBRARIES})
target_link_libraries (SegRansac ${PCL_LIBRARIES})
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})
target_link_libraries (ope_pcl ${PCL_LIBRARIES})
target_link_libraries (ply2pcd ${PCL_LIBRARIES})
target_link_libraries (pcl_correspondence ${PCL_LIBRARIES})






find_package(Boost REQUIRED COMPONENTS)
target_link_libraries (nearestKSearch pthread libpthread.so boost_thread ${PCL_LIBRARIES})
target_link_libraries (SelectPC ${PCL_LIBRARIES} pthread libpthread.so boost_thread)
target_link_libraries (ExtractPC ${PCL_LIBRARIES} pthread libpthread.so boost_thread)
target_link_libraries (Registration ${PCL_LIBRARIES} pthread libpthread.so boost_thread)
target_link_libraries (PC_splicing ${PCL_LIBRARIES} pthread libpthread.so boost_thread)
target_link_libraries (pcl_viewer ${PCL_LIBRARIES} pthread libpthread.so boost_thread)
target_link_libraries (SegRansac ${PCL_LIBRARIES} pthread libpthread.so boost_thread)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES} pthread libpthread.so boost_thread)
target_link_libraries (ope_pcl ${PCL_LIBRARIES} pthread libpthread.so boost_thread)
target_link_libraries (pcl_correspondence ${PCL_LIBRARIES} pthread libpthread.so boost_thread)



# 为使用 sophus，您需要使用find_package命令找到它
find_package( Sophus REQUIRED )
set(Sophus_LIBRARIES "/usr/local/lib/libSophus.so")
include_directories( ${Sophus_INCLUDE_DIRS} )
target_link_libraries( ICP_test ${Sophus_LIBRARIES} )
target_link_libraries(Caculate_R ${Sophus_LIBRARIES} )
target_link_libraries(ope_pcl ${Sophus_LIBRARIES} )



#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(Registration ${OpenCV_LIBRARIES})
target_link_libraries(SelectPC ${OpenCV_LIBRARIES})
target_link_libraries(PC_splicing ${OpenCV_LIBRARIES})
target_link_libraries(SegRansac ${OpenCV_LIBRARIES})
target_link_libraries(cluster_extraction ${OpenCV_LIBRARIES})
target_link_libraries(ope_pcl ${OpenCV_LIBRARIES})
target_link_libraries(pcl_correspondence ${OpenCV_LIBRARIES})
