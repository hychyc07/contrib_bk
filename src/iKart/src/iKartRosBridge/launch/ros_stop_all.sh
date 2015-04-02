ps -ef | grep -i ros 
kill `ps -ef | grep -i ros | grep -v grep | awk '{print $2}'`
