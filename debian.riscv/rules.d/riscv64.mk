build_arch	= riscv
defconfig	= defconfig
flavours	= generic
build_image	= vmlinuz.efi
kernel_file	= arch/$(build_arch)/boot/vmlinuz.efi
install_file	= vmlinuz

vdso		= vdso_install
no_dumpfile	= true

do_tools_usbip		= true
do_tools_cpupower	= true
do_tools_perf		= true
do_tools_perf_jvmti	= true
do_tools_perf_python	= true
do_tools_bpftool	= true
do_tools_rtla		= true
do_dtbs			= true
