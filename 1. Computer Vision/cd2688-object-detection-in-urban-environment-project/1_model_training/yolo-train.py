from ultralytics import YOLO

# Load a model
model = YOLO("yolov8x.pt")  # load a pretrained model (recommended for training)

# train the model
results = model.train(
    name="yolov8x",
    seed=42,
    verbose=False,
    pretrained=True,
    optimizer='auto',
    cos_lr=True,
    data="data.yaml", 
    epochs=50, 
    imgsz=640, 
    augment=True, 
    plots=True,
    # Additional training params
    # initial learning rate
    lr0=0.001,
    # final learning rate
    lrf=0.01,
    # SGD momentum/Adam beta1
    momentum=0.937,
    # optimizer weight decay
    weight_decay=0.0005,
    # warmup epochs
    warmup_epochs=3.0,
    # warmup initial momentum
    warmup_momentum=0.8,
    # warmup initial bias lr
    warmup_bias_lr=0.1,
    # Automatic Mixed Precision (AMP) training
    amp=True,
    # Augmentations
    # image HSV-Hue augmentation (fraction)
    hsv_h=0.015,
    # image HSV-Saturation augmentation (fraction)
    hsv_s=0.7,
    # image HSV-Value augmentation (fraction)
    hsv_v=0.4,
    # image rotation (+/- deg)
    degrees=0.0,
    # image translation (+/- fraction)
    translate=0.1,
    # image scale (+/- gain)
    scale=0.5,
    # image shear (+/- deg)
    shear=0.0,
    # image perspective (+/- fraction), range 0-0.001
    perspective=0.0,
    # image flip up-down (probability)
    flipud=0.0,
    # image flip left-right (probability) - will be disabled for pose estimation task
    fliplr=0.5,
    # image mosaic (probability)
    mosaic=0.0,
    # image mixup (probability)
    mixup=0.0,
    # segment copy-paste (probability)
    copy_paste=0.0
    )  

#results = model.train(data='data.yaml', epochs=25, imgsz=640, augment=True, cfg="config.yaml")

