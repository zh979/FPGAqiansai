############################################################
## Vitis HLS 工程脚本（由原工程 script.tcl 改写，路径适配本仓库结构）
## 顶层：conv    器件：xc7z020-clg400-2    目标时钟：5.5 ns
## 用法：cd build && vitis_hls -f hls_build.tcl
############################################################

open_project top
set_top conv

# 设计源文件
add_files ../src/hls/conv.h
add_files ../src/hls/conv.cpp
add_files ../src/hls/std_conv.h
add_files ../src/hls/std_conv.cpp
add_files ../src/hls/pwconv.h
add_files ../src/hls/pwconv.cpp
add_files ../src/hls/maxpool.h
add_files ../src/hls/maxpool.cpp
add_files ../src/hls/upsample.h
add_files ../src/hls/upsample.cpp

# 测试台
add_files -tb ../sim/winoconv_test.h  -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
add_files -tb ../sim/winoconv_test.cpp -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
add_files -tb ../sim/pwconv_test.h    -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
add_files -tb ../sim/pwconv_test.cpp  -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
add_files -tb ../sim/sampling_test.h  -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
add_files -tb ../sim/sampling_test.cpp -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
add_files -tb ../sim/test_tb.cpp      -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"

open_solution "solution1"
set_part {xc7z020-clg400-2}
create_clock -period 5.5 -name default
config_export -format ip_catalog -rtl verilog -version 0.0.0

csim_design
csynth_design
cosim_design
export_design -rtl verilog -format ip_catalog -version "0.0.0"
