include("openroad")

add_library(UvDRCLib
    src/UvDRC.cc
)

target_include_directories(UvDRCLib
  PUBLIC
    include
  PRIVATE
    # A side-effect of including OpenSTA swig files
    ${OPENSTA_HOME}
    ${OPENSTA_HOME}/include/sta
    src
)

target_link_libraries(UvDRCLib
  PUBLIC
    est_lib
    stt_lib
    odb
    OpenSTA
    dbSta_lib
    grt_lib
    utl_lib
    rsz_lib
)

messages(
  TARGET UvDRCLib
  OUTPUT_DIR ..
)
