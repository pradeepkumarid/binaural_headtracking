include_directories(/home/pradeep/Q4/SpatialAudio/Project/libsofa/src)
include_directories(/home/pradeep/Q4/SpatialAudio/Project/libsofa/dependencies/include)

include_directories(/home/pradeep/Q4/SpatialAudio/Project/ann_1.1.2/include/ANN)


FIND_LIBRARY(SOFA sofa PATHS /home/pradeep/Q4/SpatialAudio/Project/libsofa/lib)
FIND_LIBRARY(NETCDF netcdf PATHS /home/pradeep/Q4/SpatialAudio/Project/libsofa/dependencies/lib/linux)
FIND_LIBRARY(NETCDFCPP netcdf_c++4 PATHS /home/pradeep/Q4/SpatialAudio/Project/libsofa/dependencies/lib/linux)
FIND_LIBRARY(Z z PATHS /home/pradeep/Q4/SpatialAudio/Project/libsofa/dependencies/lib/linux)
FIND_LIBRARY(HDF5 hdf5 PATHS /home/pradeep/Q4/SpatialAudio/Project/libsofa/dependencies/lib/linux)
FIND_LIBRARY(HDF5HL hdf5_hl PATHS /home/pradeep/Q4/SpatialAudio/Project/libsofa/dependencies/lib/linux)
FIND_LIBRARY(CURL curl PATHS /home/pradeep/Q4/SpatialAudio/Project/libsofa/dependencies/lib/linux)

FIND_LIBRARY(ANN ANN /home/pradeep/Q4/SpatialAudio/Project/ann_1.1.2/lib)

TARGET_LINK_LIBRARIES (${APP_NAME} ${SOFA} ${NETCDFCPP} ${NETCDF} ${HDF5HL} ${HDF5} ${CURL} ${Z} ${ANN})
