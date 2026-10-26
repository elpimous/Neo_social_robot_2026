convertir le modele en tensorrt :

/usr/src/tensorrt/bin/trtexec \
  --onnx=/home/nvidia/qbo_ws/src/qbo_faces_tracking/tools/version-RFB-320.onnx \
  --saveEngine=/home/nvidia/qbo_ws/src/qbo_faces_tracking/tools/version-RFB-320.engine \
  --fp16 \
  --verbose
