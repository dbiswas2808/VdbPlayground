cmake_minimum_required ( VERSION 3.12.0 )

add_library ( vdb_fields_application
    application.h
)

set_target_properties ( vdb_fields_application PROPERTIES LINKER_LANGUAGE CXX )

target_link_libraries ( vdb_fields_application
    UI
)

target_include_directories ( vdb_fields_application PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/ )