while :
do

espeak "hello this is iCub and these are some expression I can do" -a 100 &

for i in {1..10}
do
	echo "ctpn time 0.1 off 0 pos (20.00 0.02 -0.03 0.04 -0.00 0.00 1.01 0.04 30.00 28.00 30.00 -11.03)" | yarp rpc /ctpservice/head/rpc
	sleep 0.1
	echo "ctpn time 0.1 off 0 pos (20.00 0.02 -0.03 0.04 0.00 0.00 22.99 -1.54 30.00 28.00 30.00 -20.00 )" | yarp rpc /ctpservice/head/rpc
	sleep 0.1
done
echo "ctpn time 0.1 off 0 pos (20.00 0.02 -0.03 0.04 -0.00 0.00 1.01 0.04 30.00 28.00 30.00 -11.03)" | yarp rpc /ctpservice/head/rpc
sleep 0.1
sleep 1
echo "ctpn time 0.2 off 0 pos (20.00 0.02 -0.03 0.04 -0.00 0.00 6.02 0.04 -19.00 28.00 -4.00 -14.99)" | yarp rpc /ctpservice/head/rpc
sleep 1
echo "ctpn time 0.2 off 0 pos (20.00 0.02 -0.03 0.04 0.00 0.00 22.99 0.04 30.00 -12.00 30.00 -20.00)" | yarp rpc /ctpservice/head/rpc
sleep 1
echo "ctpn time 0.2 off 0 pos (20.00 0.02 -0.03 0.04 -0.00 0.00 16.04 0.04 1.00 28.00 35.00 -20.00 )" | yarp rpc /ctpservice/head/rpc
sleep 1
echo "ctpn time 0.2 off 0 pos (20.00 0.02 -0.03 0.04 -0.00 0.00 16.13 0.04 35.00 28.00 0.00 -20.00 )" | yarp rpc /ctpservice/head/rpc
sleep 1
echo "ctpn time 0.2 off 0 pos (20.00 0.02 -0.03 -20.04 -0.00 0.00 21.13 -102.04 -7.00 0.00 -10.00 -20.00 )" | yarp rpc /ctpservice/head/rpc
sleep 1
echo "ctpn time 0.2 off 0 pos (20.00 0.02 -0.03  6.04 -0.00 0.00 21.13  2.04 -7.00 28.00 -10.00 -20.00 )" | yarp rpc /ctpservice/head/rpc
sleep 1
aplay amado_mio.wav &
for i in {1..2}
do
	echo "ctpn time 0.1 off 0 pos (20.00 0.02 -0.03 0.04 -0.00 0.00 1.01 0.04 30.00 28.00 30.00 -11.03)" | yarp rpc /ctpservice/head/rpc
	sleep 0.1
	echo "ctpn time 0.1 off 0 pos (20.00 0.02 -0.03 0.04 0.00 0.00 22.99 -1.54 30.00 28.00 30.00 -20.00 )" | yarp rpc /ctpservice/head/rpc
	sleep 0.1
done
echo "ctpn time 0.2 off 0 pos (20.00 0.02 -0.03 0.04 0.00 0.00 22.99 0.04 30.00 28.00 30.00 -20.00 )" | yarp rpc /ctpservice/head/rpc
sleep 0.1
for i in {1..36}
do
	echo "ctpn time 0.01 off 0 pos (20.00 0.02 -0.03 0.04 0.00 0.00 22.99 -1.54 30.00 28.00 30.00 -20.00 )" | yarp rpc /ctpservice/head/rpc
#	sleep 0.01
	echo "ctpn time 0.01 off 0 pos (20.00 0.02 -0.03 0.04 0.00 0.00 14 -1.54 30.00 28.00 30.00 -20.00 )" | yarp rpc /ctpservice/head/rpc
#	sleep 0.01
done
for i in {1..4}
do
	echo "ctpn time 0.1 off 0 pos (20.00 0.02 -0.03 0.04 -0.00 0.00 1.01 0.04 30.00 28.00 30.00 -11.03)" | yarp rpc /ctpservice/head/rpc
	sleep 0.1
	echo "ctpn time 0.1 off 0 pos (20.00 0.02 -0.03 0.04 0.00 0.00 22.99 -1.54 30.00 28.00 30.00 -20.00 )" | yarp rpc /ctpservice/head/rpc
	sleep 0.1
done
sleep 0.5
echo "ctpn time 0.2 off 0 pos (20.00 0.02 -0.03 0.04 0.00 0.00 10 -1.54 30.00 28.00 30.00 -20.00 )" | yarp rpc /ctpservice/head/rpc

sleep 1

sleep 2
done
 







