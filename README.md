nxc2620
=======

linux 2.6.23 kernel for nxc2620 dvk 3.2 (mipsel)


========================================================================

  	linux 2.6 installation for NXC2600 Platform


1. files:

	u-boot.bin			 : bootloader for nor flash.
	u-boot-nand.bin 		 : bootloader for nand flash.
	uImage.kernel			 : linux kernel.
	rootfs.img			 : initrd.
	rootfs_init.img      : tftp install.
	nxc2600-nfsroot-20090515.tar.bz2 : nfsroot system.
	nxc2600-nand-20090515.tar.bz2	 : nand root filesystem.
	rt73-cvs-2007111600.tar.bz2	 : ralink rt2571wf driver.

	linux-2.6.23-NXC2600-0.5.0-20090515-patch.bz2 : linux kernel patch
	u-boot-1.2.0-NXC2600-0.5.0-20090428-20090429-patch.bz2 : bootloader patch
	mipsel-linux-nxc2600-20080904.tar.bz2	      : toolchain



2. install bootloader:

	2.1. set uart port0 params to 57600/8N1.

	2.2. first time install when no bootloader in flash:

		use jtag to program bootloader into flash.

		for nor flash in jtag: 

		    JDI>erase 0xbfc00000 0x10000 4
		    JDI>prog 0xbfc00000 192.168.1.60 u-boot.bin

		for nand flash in jtag:

		    JDI>nerase 0 4
		    JDI>nprog 0 192.168.1.60 u-boot-nand.bin

	2.3. by default the bootloader params are:

		host ip	: 192.168.1.3
		server ip: 192.168.1.60

		change host ip:
			
		    NXC2600# set ipaddr 192.168.x.x
		    NXC2600# save

		change server ip:

		    NXC2600# set serverip 192.168.x.x
		    NXC2600# save

	2.4. if you want to upgrade bootloader in bootloader:

		  copy a new bootloader into /tftpboot in your tftp server, 
		and startup tftp server then enter:

		    NXC2600# run update_b

		NOTE: if you use nand flash and program failed in any errors,
                      just try again and again until it successful.



3. install linux kernel:

	  Before this step, you must setup your network settings, see 2.3.

	  then copy the file "uImage.kernel" into /tftpboot and
	startup your tftp server.

	  when all settings are down, you can upgrade linux kernel by:

	      NXC2600# run update_k

	  When you enter "run update_k", the tftp program
        will automatic downloading over tftp protocol, and waitting
        untill download compeleted and program into flash.

	  NOTE: if you use nand flash and program failed in any errors,
		just try again and again until it successful.

4. install initrd:

          Before this step, you must setup your network settings, see 2.3.

          then copy the file "rootfs.img" into /tftpboot and
        startup your tftp server.

          when all settings are down, you can upgrade initrd by:

               NXC2600# run update_r

          When you enter "run update_r", the tftp program
        will automatic downloading over tftp protocol, and waitting
        untill download compeleted and program into flash.

          NOTE: if you use nand flash and program failed in any errors,
                just try again and again until it successful.

5. install nfsroot

        the default nfsroot entry is "/nfsroot/nxc2600"

        #tar jxvf /path/to/nxc2600-nfsroot-20090515.tar.bz2 -C /nfsroot


6. linux bootup commands:

	6.1. normal bootup: (by default, boot to nand)

	    NXC2600# run defaultboot

	6.2. nfs root bootup:

	    NXC2600# run nfsboot

	  by default the nfsroot entry is "serverip:/nfsroot/nxc2600".
	change entry by:

	    NXC2600# set nfsroot /path/to/nfsroot
	    NXC2600# save


	6.3. boot into nand:

	    NXC2600# run nandboot



7. install root filesystem:

        7.1. sd card root filesystem is not longer support.
        7.2. in nand flash, boot into nfsroot first :

            7.2.1. untar the file "nxc2600-nand-20090515.tar.bz2" on server side.

                #tar jxvf /path/to/nxc2600-nand-20090515.tar.bz2 -C /nfsroot/nxc2600

            7.2.2. copy all files into nand on nxc2600 dvb, you need to bootup into nfsroot mode to do this.

                #/tool/install_nand.sh


8. install toolchain:

	    #tar jxvf /path/to/mipsel-linux-nxc2600-20080904.tar.bz2 -C /
	    #set PATH=$PATH:/opt/toolchain/mipsel-linux/bin

9. create a new linux kernel:

	download kernel(2.6.23) form http://kernel.org then
	
	    #cd /path/to/linux-2.6.23
	    #bzcat /path/to/linux-2.6.23-NXC2600-0.5.0-20090515-patch.bz2 |patch -p1
	    #cp arch/mips/defconfig .config
	    #make menuconfig
	       select LCD type
	    #make uImage

	after "make uImage", the default setting will copy "arch/mips/boot/uImage" to /tftpboot/uImage.kernel".




10. create a new bootloader:

	download u-boot(1.2.0) form http://www.denx.de/wiki/UBoot

	    #cd /path/to/u-boot-1.2.0
	    #bzcat /path/to/u-boot-1.2.0-NXC2600-0.5.0-20090428-20090429-patch.bz2 |patch -p1

	for nor flash:

	    #cd /path/to/u-boot-1.2.0
	    #make distclean
	    #make nxc2600_dvb2_config
	    #make
	    #cp u-boot.bin /tftpboot/


	for nand flash:

	    #cd /path/to/u-boot-1.2.0
	    #make distclean
	    #make nxc2600_dvb3_nand_config
	    #make
	    #cp u-boot-nand.bin /tftpboot/

