# export_xsa.tcl -- 重新导出硬件平台 (Vivado 2019.2)
#
# 什么时候需要跑这个脚本
#   改动了块设计或重跑了实现之后。XSA 里内嵌的是导出那一刻的 bitstream，
#   若它与 build/reports/ 下的资源时序报告不是同一次实现的产物，
#   平台与报告数字就对不上了，需要重新导出一次。
#
# 不需要重新综合实现
#   XSA = 硬件描述 + bitstream。硬件描述决定 xparameters.h，它没变，
#   所以 PS 侧代码一行都不用改。这个脚本只是把已经布线完成的 impl_1
#   重新打包一次，正常情况几秒钟。
#
# 用法
#   1. Vivado 2019.2 打开你的 acc.xpr
#   2. 底部 Tcl Console 里执行:
#          source E:/.../build/export_xsa.tcl
#      (路径按脚本实际位置填，用正斜杠)
#
#   脚本直接作用于当前打开的工程，不需要在里面填任何路径。
#
# 如果脚本报 NEEDS_REFRESH = 1
#   说明源文件时间戳比实现结果新，Vivado 认为实现已过期。
#   此时导出的仍是上一次布线的 bitstream。若要让 XSA 与报告严格对应，
#   需先 Generate Bitstream 重跑，而重跑后布线种子不同，
#   WNS 与资源数字会有小幅变化 —— 必须重新截图 Project Summary
#   并更新报告里的 +0.189 ns 等数值，不要沿用旧数字。

# ---- 1. 确认有工程打开 ----
if {[catch {current_project} proj]} {
    puts ""
    puts "ERROR: 当前没有打开任何工程。"
    puts "       请先在 Vivado 里 File -> Open Project 打开 acc.xpr，再 source 本脚本。"
    puts ""
    return
}
puts ""
puts "当前工程 : $proj"
puts "工程路径 : [get_property DIRECTORY [current_project]]"

# ---- 2. 查实现状态 ----
if {[llength [get_runs -quiet impl_1]] == 0} {
    puts "ERROR: 工程里没有 impl_1 这个实现 run。"
    return
}
set impl_run [get_runs impl_1]
set st [get_property STATUS        $impl_run]
set pr [get_property PROGRESS      $impl_run]
set nd [get_property NEEDS_REFRESH $impl_run]

puts ""
puts "=================================================="
puts " impl_1 状态  : $st"
puts " 进度         : $pr"
puts " 需要刷新     : $nd"
puts "=================================================="
puts ""

if {$pr ne "100%"} {
    puts "ERROR: impl_1 没有完成布线，无法导出带 bitstream 的平台。"
    puts "       请先 Generate Bitstream。"
    return
}

if {$nd} {
    puts "WARNING: Vivado 认为 impl_1 已过期(源文件有改动)。"
    puts "         现在导出的 bitstream 仍是上一次布线的结果。"
    puts "         若要让报告数字与 XSA 严格对应，请先重跑实现，"
    puts "         并在重跑后重新采集资源与时序数字。"
    puts ""
}

# ---- 3. 打开实现结果并导出 ----
# 已经打开就不用再开一次，重复 open_run 会报错
if {[catch {current_run -implementation} cur] || $cur ne $impl_run} {
    open_run impl_1
}

# 输出到本脚本所在目录的 hw/ 子目录
set script_dir [file dirname [file normalize [info script]]]
set out [file join $script_dir hw design_1_wrapper.xsa]
file mkdir [file dirname $out]

# -fixed       : 固定平台(无动态区域重配置)，裸机应用用这个
# -include_bit : 把 bitstream 一起打进去，Vitis 才能一键烧写
# -force       : 覆盖已有文件
write_hw_platform -fixed -include_bit -force $out

puts ""
puts "已导出 -> $out"
puts ""
puts "接下来在 Vitis 2019.2 里:"
puts "  1. 左侧 Explorer 选中 platform 工程(ZLZ)"
puts "  2. 右键 -> Update Hardware Specification，选这份新 XSA"
puts "  3. 右键 platform -> Build Project"
puts "  4. 右键 应用工程(test) -> Clean Project，再 Build Project"
puts ""
puts "因为硬件描述(design_1.hwh)没变，xparameters.h 不会变，代码无需改动。"
puts "Build 完如果没有报错，就说明平台替换成功。"
puts ""
