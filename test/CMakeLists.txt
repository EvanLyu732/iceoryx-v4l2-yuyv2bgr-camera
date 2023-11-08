add_executable(dma_recevier
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cc
)

target_include_directories(dma_recevier PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    # ${PROJECT_SOURCE_DIR}/externel
    # ${PROJECT_SOURCE_DIR}/externel/jetson_multimedia_api/include
)

target_link_libraries(dma_recevier PRIVATE
    ${OpenCV_LIBS}
    # nv_api_common
    # nvbufsurface
    # nvbufsurftransform
    # -lpthread
    # -lrt
    iceoryx_posh::iceoryx_posh
    iceoryx_hoofs::iceoryx_hoofs
)