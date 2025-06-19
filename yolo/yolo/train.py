from ultralytics import YOLO

model = YOLO('/home/paula/security_bot_ws/src/yolo/weights/best.pt')

model.train(
    data = '/home/paula/security_bot_ws/src/yolo/data/data.yaml',
    epochs = 50,
    imgsz = 640,
    batch = 8
)

