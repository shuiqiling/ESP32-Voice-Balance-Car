set(src_dirs
            SystemState
            PID
            DebugUART
            Filter
)

set(include_dirs
            SystemState
            PID
            DebugUART
            Filter
)

set(requires_dirs
            driver
            Config
            BSP
            json
            freertos
)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires_dirs})
