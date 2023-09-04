ppids=$(ps -A --format pid,cmd | grep 'roslaunch' | awk '{$1=$1};1' | cut -d " " -f 1) 

ppid=$(echo $ppids | cut -d " " -f1)
pkill -P $ppid
