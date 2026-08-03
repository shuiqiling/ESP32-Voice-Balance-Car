set(src_dirs
            Config_MOTOR
            Config_ENCODER
            Config_IIC
            Config_MPU6050
            Config_MIC
)

set(include_dirs
            Config_MOTOR
            Config_ENCODER
            Config_IIC
            Config_MPU6050
            Config_MIC
)

set(requires_dirs


)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires_dirs})
