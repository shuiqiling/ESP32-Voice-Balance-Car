set(src_dirs
            SER_BALANCECAR
            SER_AIWORK
)

set(include_dirs
            SER_BALANCECAR
            SER_AIWORK
)

set(requires_dirs
            MYHAL
            driver
            Config
            Core
            NET
)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires_dirs})
