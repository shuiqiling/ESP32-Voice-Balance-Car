set(src_dirs
        MYHAL_MOTOR
        MYHAL_ENCODER
        MYHAL_MPU6050
        IIC
        MYHAL_MIC
)

set(include_dirs
        MYHAL_MOTOR
        MYHAL_ENCODER
        MYHAL_MPU6050
        IIC
        MYHAL_MIC
)

set(requires_dirs
            driver
            BSP
            Config
            Core

)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires_dirs})
