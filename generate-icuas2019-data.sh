cd bin/
for i in `seq 10 10 50`
do
    ./generate-data -t $i -n 5 -s 1
done
