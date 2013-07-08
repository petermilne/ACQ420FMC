#!/bin/sh



mkdir /usr/local
mount -t ramfs local /usr/local

export PATH=$PATH:/usr/local/bin
export LD_LIBRARY_PATH=/usr/local/lib/


cat - >>/etc/profile <<EOF
export PATH=$PATH
export LD_LIBRARY_PATH=/usr/local/lib/
EOF


for package in /mnt/packages/??-*.tgz
do
	tar xvzf $package -C /
	FN=$(basename ${package})
	FNP=${FN%-*}
	PN=${FNP#*-}
	if [ -f /usr/local/init/${PN}.init ]; then
		/usr/local/init/${PN}.init
	fi
done
	
