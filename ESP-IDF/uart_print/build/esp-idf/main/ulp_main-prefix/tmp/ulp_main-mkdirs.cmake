# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Espressif/frameworks/esp-idf-v5.5/components/ulp/cmake")
  file(MAKE_DIRECTORY "C:/Espressif/frameworks/esp-idf-v5.5/components/ulp/cmake")
endif()
file(MAKE_DIRECTORY
  "C:/Users/ACER/Desktop/My-Learning-Journey/ESP-IDF/uart_print/build/esp-idf/main/ulp_main"
  "C:/Users/ACER/Desktop/My-Learning-Journey/ESP-IDF/uart_print/build/esp-idf/main/ulp_main-prefix"
  "C:/Users/ACER/Desktop/My-Learning-Journey/ESP-IDF/uart_print/build/esp-idf/main/ulp_main-prefix/tmp"
  "C:/Users/ACER/Desktop/My-Learning-Journey/ESP-IDF/uart_print/build/esp-idf/main/ulp_main-prefix/src/ulp_main-stamp"
  "C:/Users/ACER/Desktop/My-Learning-Journey/ESP-IDF/uart_print/build/esp-idf/main/ulp_main-prefix/src"
  "C:/Users/ACER/Desktop/My-Learning-Journey/ESP-IDF/uart_print/build/esp-idf/main/ulp_main-prefix/src/ulp_main-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/ACER/Desktop/My-Learning-Journey/ESP-IDF/uart_print/build/esp-idf/main/ulp_main-prefix/src/ulp_main-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/ACER/Desktop/My-Learning-Journey/ESP-IDF/uart_print/build/esp-idf/main/ulp_main-prefix/src/ulp_main-stamp${cfgdir}") # cfgdir has leading slash
endif()
