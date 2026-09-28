# UUV models

Do not commit private Roboflow API keys here.

For the Ultralytics backend, place the deployed model here, for example `uuv_yolo.pt` or a TensorRT `uuv_yolo.engine` built/validated on the target Jetson.

For the Roboflow backend, leave weights managed by the local Roboflow Inference server and set `roboflow_model_id` in the launch command or YAML.
