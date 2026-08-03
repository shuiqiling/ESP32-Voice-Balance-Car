set(src_dirs
            BSP_MOTOR
            BSP_ENCODER
            BSP_IIC
            BSP_MIC
)

set(include_dirs
            BSP_MOTOR
            BSP_ENCODER
            BSP_IIC
            BSP_MIC
)

set(requires_dirs
            driver
            Config

)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires_dirs})
