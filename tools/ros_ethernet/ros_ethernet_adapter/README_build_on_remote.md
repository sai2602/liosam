# Building on remote device

* Set environment variables
```
# Source IP, user, directory
SRCIP=192.168.7.106
SRCUSER=robert
SRCDIR=/media/robert/Shared/Git/anavs-development/vision
# Destination IP, user, directory
DESTIP=192.168.7.163
DESTUSER=anavs
DESTDIR=/home/${DESTUSER}/Development
```

* Copy project to remote device (project sources)
```
cd ${SRCDIR}
ssh $DESTUSER@$DESTIP "mkdir -p ${DESTDIR}" && scp -r ros_ethernet_adapter ${DESTUSER}@${DESTIP}:${DESTDIR}
```

* Build project on remote device (login via ssh to device before)
```
ssh $DESTUSER@$DESTIP
cd ${DESTDIR}/ros_ethernet_adapter
./build_Release.sh
```

* (optional) Copy ARM binaries back to source system
```
cd ${DESTDIR}/ros_ethernet_adapter
scp bin/Release/anavs_ros_ethernet_server ${SRCUSER}@${SRCIP}:${SRCDIR}/ros_ethernet_adapter/bin/Release/anavs_ros_ethernet_server_remote

cd ${DESTDIR}/ros_ethernet_adapter/client
scp bin/Release/anavs_ros_ethernet_client ${SRCUSER}@${SRCIP}:${SRCDIR}/ros_ethernet_adapter/clinet/bin/Release/anavs_ros_ethernet_client_remote
```