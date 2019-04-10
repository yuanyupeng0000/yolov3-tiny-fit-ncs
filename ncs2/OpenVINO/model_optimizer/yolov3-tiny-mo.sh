#python3 mo.py --input_model 20190218182235_TinyYoloV3NCS.caffemodel  --data_type FP16 --output_dir FP16 --scale_value data[255,255,255]
caffemodel=$1
python3 /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/model_optimizer/mo.py --input_model $caffemodel --data_type FP16 --output_dir /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/model_optimizer/FP16 --scale_value data[255,255,255]
