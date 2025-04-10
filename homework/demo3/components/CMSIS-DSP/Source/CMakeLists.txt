cmake_minimum_required (VERSION 3.14)
cmake_policy(SET CMP0077 NEW)
project(CMSISDSP)

# DSP Sources
SET(DSP ${CMAKE_CURRENT_SOURCE_DIR}/..)

option(NEON "Neon acceleration" OFF)
option(NEONEXPERIMENTAL "Neon experimental acceleration" OFF)
option(HELIUMEXPERIMENTAL "Helium experimental acceleration" OFF)
option(LOOPUNROLL "Loop unrolling" ON)
option(ROUNDING "Rounding" OFF)
option(MATRIXCHECK "Matrix Checks" OFF)
option(HELIUM "Helium acceleration (MVEF and MVEI supported)" OFF)
option(MVEF "MVEF intrinsics supported" OFF)
option(MVEI "MVEI intrinsics supported" OFF)
option(MVEFLOAT16 "Float16 MVE intrinsics supported" OFF)
option(DISABLEFLOAT16 "Disable building float16 kernels" OFF)
option(HOST "Build for host" OFF)
option(AUTOVECTORIZE "Prefer autovectorizable code to one using C intrinsics" OFF)

# Select which parts of the CMSIS-DSP must be compiled.
# There are some dependencies between the parts but they are not tracked
# by this cmake. So, enabling some functions may require to enable some
# other ones.
option(BASICMATH            "Basic Math Functions"              ON)
option(COMPLEXMATH          "Complex Math Functions"            ON)
option(CONTROLLER           "Controller Functions"              ON)
option(FASTMATH             "Fast Math Functions"               ON)
option(FILTERING            "Filtering Functions"               ON)
option(MATRIX               "Matrix Functions"                  ON)
option(STATISTICS           "Statistics Functions"              ON)
option(SUPPORT              "Support Functions"                 ON)
option(TRANSFORM            "Transform Functions"               ON)
option(SVM                  "Support Vector Machine Functions"  ON)
option(BAYES                "Bayesian Estimators"               ON)
option(DISTANCE             "Distance Functions"                ON)
option(INTERPOLATION        "Interpolation Functions"           ON)
option(QUATERNIONMATH       "Quaternion Math Functions"         ON)

# When OFF it is the default behavior : all tables are included.
option(CONFIGTABLE          "Configuration of table allowed"    OFF)

# When CONFIGTABLE is ON, select if all interpolation tables must be included
option(ALLFAST              "All interpolation tables included" OFF)
# When CONFIGTABLE is ON, select if all FFT tables must be included
option(ALLFFT               "All fft tables included"           OFF)

# Features which require inclusion of a data table.
# Since some tables may be big, the corresponding feature can be
# disabled.
# Those options are taken into account only when CONFIGTABLE is ON
option(ARM_COS_F32          "cos f32"                           OFF)
option(ARM_COS_Q31          "cos q31"                           OFF)
option(ARM_COS_Q15          "cos q15"                           OFF)
option(ARM_SIN_F32          "sin f32"                           OFF)
option(ARM_SIN_Q31          "sin q31"                           OFF)
option(ARM_SIN_Q15          "sin q15"                           OFF)
option(ARM_SIN_COS_F32      "sin cos f32"                       OFF)
option(ARM_SIN_COS_Q31      "sin cos q31"                       OFF)

option(ARM_LMS_NORM_Q31     "lms norm q31"                      OFF)
option(ARM_LMS_NORM_Q15     "lms norm q15"                      OFF)

option(CFFT_F64_16          "cfft f64 16"                       OFF)
option(CFFT_F64_32          "cfft f64 32"                       OFF)
option(CFFT_F64_64          "cfft f64 64"                       OFF)
option(CFFT_F64_128         "cfft f64 128"                      OFF)
option(CFFT_F64_256         "cfft f64 256"                      OFF)
option(CFFT_F64_512         "cfft f64 512"                      OFF)
option(CFFT_F64_1024        "cfft f64 1024"                     OFF)
option(CFFT_F64_2048        "cfft f64 2048"                     OFF)
option(CFFT_F64_4096        "cfft f64 4096"                     OFF)

option(CFFT_F32_16          "cfft f32 16"                       OFF)
option(CFFT_F32_32          "cfft f32 32"                       OFF)
option(CFFT_F32_64          "cfft f32 64"                       OFF)
option(CFFT_F32_128         "cfft f32 128"                      OFF)
option(CFFT_F32_256         "cfft f32 256"                      OFF)
option(CFFT_F32_512         "cfft f32 512"                      OFF)
option(CFFT_F32_1024        "cfft f32 1024"                     OFF)
option(CFFT_F32_2048        "cfft f32 2048"                     OFF)
option(CFFT_F32_4096        "cfft f32 4096"                     OFF)

option(CFFT_Q31_16          "cfft q31 16"                       OFF)
option(CFFT_Q31_32          "cfft q31 32"                       OFF)
option(CFFT_Q31_64          "cfft q31 64"                       OFF)
option(CFFT_Q31_128         "cfft q31 128"                      OFF)
option(CFFT_Q31_256         "cfft q31 256"                      OFF)
option(CFFT_Q31_512         "cfft q31 512"                      OFF)
option(CFFT_Q31_1024        "cfft q31 1024"                     OFF)
option(CFFT_Q31_2048        "cfft q31 2048"                     OFF)
option(CFFT_Q31_4096        "cfft q31 4096"                     OFF)

option(CFFT_Q15_16          "cfft q15 16"                       OFF)
option(CFFT_Q15_32          "cfft q15 32"                       OFF)
option(CFFT_Q15_64          "cfft q15 64"                       OFF)
option(CFFT_Q15_128         "cfft q15 128"                      OFF)
option(CFFT_Q15_256         "cfft q15 256"                      OFF)
option(CFFT_Q15_512         "cfft q15 512"                      OFF)
option(CFFT_Q15_1024        "cfft q15 1024"                     OFF)
option(CFFT_Q15_2048        "cfft q15 2048"                     OFF)
option(CFFT_Q15_4096        "cfft q15 4096"                     OFF)

option(RFFT_FAST_F32_32     "rfft fast f32 32"                  OFF)
option(RFFT_FAST_F32_64     "rfft fast f32 64"                  OFF)
option(RFFT_FAST_F32_128    "rfft fast f32 128"                 OFF)
option(RFFT_FAST_F32_256    "rfft fast f32 256"                 OFF)
option(RFFT_FAST_F32_512    "rfft fast f32 512"                 OFF)
option(RFFT_FAST_F32_1024   "rfft fast f32 1024"                OFF)
option(RFFT_FAST_F32_2048   "rfft fast f32 2048"                OFF)
option(RFFT_FAST_F32_4096   "rfft fast f32 4096"                OFF)


option(RFFT_F32_128         "rfft f32 128"                      OFF)
option(RFFT_F32_512         "rfft f32 512"                      OFF)
option(RFFT_F32_2048        "rfft f32 2048"                     OFF)
option(RFFT_F32_8192        "rfft f32 8192"                     OFF)

option(RFFT_FAST_F64_32     "rfft fast f64 32"                  OFF)
option(RFFT_FAST_F64_64     "rfft fast f64 64"                  OFF)
option(RFFT_FAST_F64_128    "rfft fast f64 128"                 OFF)
option(RFFT_FAST_F64_256    "rfft fast f64 256"                 OFF)
option(RFFT_FAST_F64_512    "rfft fast f64 512"                 OFF)
option(RFFT_FAST_F64_1024   "rfft fast f64 1024"                OFF)
option(RFFT_FAST_F64_2048   "rfft fast f64 2048"                OFF)
option(RFFT_FAST_F64_4096   "rfft fast f64 4096"                OFF)


option(RFFT_F64_128         "rfft f64 128"                      OFF)
option(RFFT_F64_512         "rfft f64 512"                      OFF)
option(RFFT_F64_2048        "rfft f64 2048"                     OFF)
option(RFFT_F64_8192        "rfft f64 8192"                     OFF)

option(RFFT_FAST_F16_32     "rfft fast f16 32"                  OFF)
option(RFFT_FAST_F16_64     "rfft fast f16 64"                  OFF)
option(RFFT_FAST_F16_128    "rfft fast f16 128"                 OFF)
option(RFFT_FAST_F16_256    "rfft fast f16 256"                 OFF)
option(RFFT_FAST_F16_512    "rfft fast f16 512"                 OFF)
option(RFFT_FAST_F16_1024   "rfft fast f16 1024"                OFF)
option(RFFT_FAST_F16_2048   "rfft fast f16 2048"                OFF)
option(RFFT_FAST_F16_4096   "rfft fast f16 4096"                OFF)

option(RFFT_Q31_32          "rfft q31 32"                       OFF)
option(RFFT_Q31_64          "rfft q31 64"                       OFF)
option(RFFT_Q31_128         "rfft q31 128"                      OFF)
option(RFFT_Q31_256         "rfft q31 256"                      OFF)
option(RFFT_Q31_512         "rfft q31 512"                      OFF)
option(RFFT_Q31_1024        "rfft q31 1024"                     OFF)
option(RFFT_Q31_2048        "rfft q31 2048"                     OFF)
option(RFFT_Q31_4096        "rfft q31 4096"                     OFF)
option(RFFT_Q31_8192        "rfft q31 8192"                     OFF)

option(RFFT_Q15_32          "rfft q15 32"                       OFF)
option(RFFT_Q15_64          "rfft q15 64"                       OFF)
option(RFFT_Q15_128         "rfft q15 128"                      OFF)
option(RFFT_Q15_256         "rfft q15 256"                      OFF)
option(RFFT_Q15_512         "rfft q15 512"                      OFF)
option(RFFT_Q15_1024        "rfft q15 1024"                     OFF)
option(RFFT_Q15_2048        "rfft q15 2048"                     OFF)
option(RFFT_Q15_4096        "rfft q15 4096"                     OFF)
option(RFFT_Q15_8192        "rfft q15 8192"                     OFF)

option(DCT4_F32_128          "dct4 f32 128"                     OFF)
option(DCT4_F32_512          "dct4 f32 512"                     OFF)
option(DCT4_F32_2048         "dct4 f32 2048"                    OFF)
option(DCT4_F32_8192         "dct4 f32 8192"                    OFF)

option(DCT4_Q31_128          "dct4 q31 128"                     OFF)
option(DCT4_Q31_512          "dct4 q31 512"                     OFF)
option(DCT4_Q31_2048         "dct4 q31 2048"                    OFF)
option(DCT4_Q31_8192         "dct4 q31 8192"                    OFF)

option(DCT4_Q15_128          "dct4 q15 128"                     OFF)
option(DCT4_Q15_512          "dct4 q15 512"                     OFF)
option(DCT4_Q15_2048         "dct4 q15 2048"                    OFF)
option(DCT4_Q15_8192         "dct4 q15 8192"                    OFF)

option(ARM_CFFT_RADIX2_Q15   "deprecated q15 radix2 cfft"       OFF)
option(ARM_CFFT_RADIX4_Q15   "deprecated q15 radix4 cfft"       OFF)

option(ARM_CFFT_RADIX2_Q31   "deprecated q31 radix2 cfft"       OFF)
option(ARM_CFFT_RADIX4_Q31   "deprecated q31 radix4 cfft"       OFF)

###########################
#
# CMSIS DSP
#
###########################



add_library(CMSISDSP STATIC)


if (BASICMATH)
  include(BasicMathFunctions/Config.cmake)
endif()

if (COMPLEXMATH)
  include(ComplexMathFunctions/Config.cmake)
endif()

if (QUATERNIONMATH)
  include(QuaternionMathFunctions/Config.cmake)
endif()

if (CONTROLLER)
  include(ControllerFunctions/Config.cmake)
  # Fast tables inclusion is allowed
  if (CONFIGTABLE)
    target_compile_definitions(CMSISDSP PUBLIC ARM_FAST_ALLOW_TABLES)
  endif()
endif()

if (FASTMATH)
  include(FastMathFunctions/Config.cmake)
  # Fast tables inclusion is allowed
  if (CONFIGTABLE)
    target_compile_definitions(CMSISDSP PUBLIC ARM_FAST_ALLOW_TABLES)
  endif()
endif()

if (FILTERING)
  include(FilteringFunctions/Config.cmake)
  # Fast tables inclusion is allowed
  if (CONFIGTABLE)
    target_compile_definitions(CMSISDSP PUBLIC ARM_FAST_ALLOW_TABLES)
  endif()
endif()

if (MATRIX)
  include(MatrixFunctions/Config.cmake)
endif()

if (STATISTICS)
  include(StatisticsFunctions/Config.cmake)
endif()

if (SUPPORT)
  include(SupportFunctions/Config.cmake)
endif()

if (TRANSFORM)
  include(TransformFunctions/Config.cmake)
  # FFT tables inclusion is allowed
  if (CONFIGTABLE)
    target_compile_definitions(CMSISDSP PUBLIC ARM_FFT_ALLOW_TABLES)
  endif()
endif()

if (FILTERING OR CONTROLLER OR FASTMATH OR TRANSFORM OR SVM OR DISTANCE)
  include(CommonTables/Config.cmake)
  if (TRANSFORM)
    # FFT tables inclusion is allowed
    if (CONFIGTABLE)
      target_compile_definitions(CMSISDSP PUBLIC ARM_FFT_ALLOW_TABLES)
    endif()
  endif()
  if (FILTERING OR CONTROLLER OR FASTMATH)
    # Select which tables to include
    if (CONFIGTABLE)
      target_compile_definitions(CMSISDSP PUBLIC ARM_FAST_ALLOW_TABLES)
    endif()
  endif()
  # Common project is adding ComputeLibrary tables used by SVM and Distance
  # when NEon is ON.
endif()

if (SVM)
  include(SVMFunctions/Config.cmake)
endif()

if (BAYES)
  include(BayesFunctions/Config.cmake)
endif()

if (DISTANCE)
  include(DistanceFunctions/Config.cmake)
endif()

if (INTERPOLATION)
  include(InterpolationFunctions/Config.cmake)
endif()

### Includes
target_include_directories(CMSISDSP PUBLIC "${DSP}/Include")
if (DEFINED CMSISCORE)
target_include_directories(CMSISDSP PUBLIC "${CMSISCORE}/Include")
endif()

include(configDsp.cmake)
include(fft.cmake)
include(interpol.cmake)

configDsp(CMSISDSP)
fft(CMSISDSP)
interpol(CMSISDSP)
