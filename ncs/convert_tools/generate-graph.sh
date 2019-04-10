caffemodel=$1
pre=${caffemodel%.*}
echo $pre 
mvNCCompile ../yolov3-tiny-ncs-without-last-maxpool.prototxt -w ./$caffemodel -s 12 -o $pre.graph_416 -ec
