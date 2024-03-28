export CROSSCOMPILE_DIR=1
if [ $1 = "GNOME" ] || [ $1 = "UBI_DEV" ]
then
export PATH=/usr/local/IMX6-Qt-4.7.4/Qt-X11-ARM-Target/bin:/u02/tools/gcc-4.6.2-glibc-2.13-linaro-multilib-2011.12/fsl-linaro-toolchain/bin:$PATH
echo "Exported the path for GNOME or UBI Device.... "

else
export PATH=$PATH:/opt/freescale/usr/local/gcc-4.1.2-glibc-2.5-nptl-3/arm-none-linux-gnueabi/bin/
fi

