# components/ED_OPT3001/CMakeLists.txt

# Register the component with the build system
idf_component_register(
    SRCS
        "ED_OPT3001.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_timer
        freertos
    PRIV_REQUIRES
        ED_SYS
)

# Optionally, set component‑specific compile options
# target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)

# If your component uses C++ features, ensure the compiler is set to C++
# set(COMPONENT_ADDITIONAL_CLEAN_FILES "")