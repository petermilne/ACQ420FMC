#!/bin/sh

NSAM=4194304

build_hb() {
	MAJOR=$1
	NAME=$2

	HBDIR=/dev/${NAME}.hb
	mkdir $HBDIR
	for hb in $(seq 0 $(cat /sys/module/acq420FMC/parameters/nbuffers))
	do
		NN=$(printf "$HBDIR/%02d" $hb)
		let minor="100+$hb"
		mknod $NN c $MAJOR $minor		
	done
}
build_nodes() {
	finit=0
	while [ $finit -eq 0 ]; do
		read M_DEV
		if [ "$M_DEV" = "" ]; then
			finit=1;
		else
			MAJOR=${M_DEV% *}
			NAME=${M_DEV#* }		
			ID=${M_DEV#*.}

			mknod /dev/ACQ420_FMC${ID} c ${MAJOR} 0
			mknod /dev/${NAME} c ${MAJOR} 0
			mknod /dev/${NAME}.c c ${MAJOR} 1
			mknod /dev/${NAME}.histo c ${MAJOR} 2
			build_hb ${MAJOR} ${NAME}
		fi
	done
}


lsmod | grep -q acq420FMC
notloaded=$?

build_knobs() {
	let ix=0
	for dir in /sys/bus/platform/devices/*FMC
	do
		ln -s $dir/ /dev/acq420.${ix}.knobs

		echo 200 >/dev/acq420.${ix}.knobs/clkdiv
		echo 0000 >/dev/acq420.${ix}.knobs/gains
		echo 1 >/dev/acq420.${ix}.knobs/simulate
		let ix=$ix+1
	done
}



if [ $notloaded -ne 0 ]; then
	if [ -f /mnt/acq420FMC.ko ]; then
		cp /mnt/acq420FMC.ko /lib/modules/d-tacq/acq420FMC.ko
	fi

	mount -t debugfs none /sys/kernel/debug/
	echo file acq420FMC_drv.c +p > /sys/kernel/debug/dynamic_debug/control
	insmod /lib/modules/d-tacq/acq420FMC.ko
	grep acq420 /proc/devices |  build_nodes
	build_knobs
fi

inetd /mnt/inetd.conf


