import os
import argparse
# import tensorflow as tf
import cv2
from multiprocessing.dummy import Pool
import os
import shutil
from tqdm import tqdm

def parse_func(serialized_example):
    features = tf.io.parse_single_example(
        serialized_example,
        features={
            'image/height': tf.io.FixedLenFeature([], tf.int64),
            'image/width': tf.io.FixedLenFeature([], tf.int64),
            'image/encoded': tf.io.FixedLenFeature([], tf.string),
            'image/format': tf.io.FixedLenFeature([], tf.string),
            'image/object/bbox/xmin': tf.io.VarLenFeature(tf.float32),
            'image/object/bbox/xmax': tf.io.VarLenFeature(tf.float32),
            'image/object/bbox/ymin': tf.io.VarLenFeature(tf.float32),
            'image/object/bbox/ymax': tf.io.VarLenFeature(tf.float32),
            'image/object/class/label': tf.io.VarLenFeature(tf.int64)
        })
    return features


def read_record(record_file):
    dataset = tf.data.TFRecordDataset([record_file])
    dataset = dataset.map(parse_func)
    return dataset


def convert_tfrecord_to_yolo(input_folder, output_folder, obj_names_file):
    # Updated label_map to map tfrecord labels (int) to YOLO class IDs (int)
    label_map = {1: 0, 2: 1, 4: 2}  # Maps tfrecord label to YOLO index
    class_names = ['vehicle', 'pedestrian', 'cyclist']  # Class names for YOLO

    # Create output folder if it does not exist
    os.makedirs(output_folder, exist_ok=True)

    # Write the object class names to the obj.names file
    with open(obj_names_file, 'w') as obj_file:
        for name in class_names:
            obj_file.write(name + '\n')

    # Process each tfrecord file in the input folder
    files = [os.path.join(input_folder, f) for f in os.listdir(input_folder) if f.endswith('.tfrecord')]
    for file in files:
        dataset = read_record(file)
        for i, features in enumerate(tqdm(dataset) if use_tqdm else dataset):
            image = features['image/encoded'].numpy()
            image_format = features['image/format'].numpy().decode('utf-8')
            height = features['image/height'].numpy()
            width = features['image/width'].numpy()
            xmin = tf.sparse.to_dense(features['image/object/bbox/xmin']).numpy()
            xmax = tf.sparse.to_dense(features['image/object/bbox/xmax']).numpy()
            ymin = tf.sparse.to_dense(features['image/object/bbox/ymin']).numpy()
            ymax = tf.sparse.to_dense(features['image/object/bbox/ymax']).numpy()
            labels = tf.sparse.to_dense(features['image/object/class/label']).numpy()

            # Create image and label paths
            image_path = os.path.join(output_folder, f'{os.path.splitext(os.path.basename(file))[0]}_{i}')
            with open(f'{image_path}.txt', 'w') as label_file:
                for x1, x2, y1, y2, label in zip(xmin, xmax, ymin, ymax, labels):
                    x_center = ((x1 + x2) / 2) * width
                    y_center = ((y1 + y2) / 2) * height
                    bbox_width = (x2 - x1) * width
                    bbox_height = (y2 - y1) * height
                    class_id = label_map.get(label, -1)  # Using get to handle missing labels safely
                    if class_id != -1:  # Only write valid class IDs
                        label_file.write(f'{class_id} {x_center/width} {y_center/height} {bbox_width/width} {bbox_height/height}\n')

            # Save the image
            with open(f'{image_path}.{image_format}', 'wb') as img_file:
                img_file.write(image)

def draw_bboxes_and_save(image_dir, output_dir):
    labels = [0, 1, 2]
    class_names = ['vehicle', 'pedestrian', 'cyclist']
    colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255)]  # Green for vehicle, Blue for pedestrian, Red for cyclist

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    for filename in tqdm(os.listdir(image_dir+"/images")):
        if filename.endswith('.jpg'):
            image_path = os.path.join(image_dir, "images", filename)
            label_path = os.path.join(image_dir, "labels", filename.replace('.jpg', '.txt'))

            # Read image
            image = cv2.imread(image_path)
            if image is None:
                continue
            height, width, _ = image.shape

            # Check if corresponding label file exists
            if os.path.exists(label_path):
                with open(label_path, 'r') as file:
                    for line_number, line in enumerate(file, start=1):
                        parts = line.strip().split()
                        class_id = int(parts[0])
                        x_center = float(parts[1]) * width
                        y_center = float(parts[2]) * height
                        bbox_width = float(parts[3]) * width
                        bbox_height = float(parts[4]) * height

                        # Convert from center coordinates to rectangle coordinates
                        xmin = int(x_center - bbox_width / 2)
                        ymin = int(y_center - bbox_height / 2)
                        xmax = int(x_center + bbox_width / 2)
                        ymax = int(y_center + bbox_height / 2)

                        # Get the color for the current class
                        color = colors[class_id] if class_id < len(colors) else (255, 255, 255)  # Default to white if unknown
                        # Draw rectangle on the image
                        cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color, 1)
                        # Use class name instead of class id for display
                        label_text = f'{class_names[class_id]} (Line {line_number})' if class_id < len(class_names) else f'Unknown (Line {line_number})'
                        # label_text = class_names[class_id] if class_id < len(class_names) else 'Unknown'
                        cv2.putText(image, label_text, (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)

            # Save the annotated image
            output_path = os.path.join(output_dir, filename)
            cv2.imwrite(output_path, image)

input_folder = "../data/tfrecord/val"
output_folder = "../data/yolo/val"
yolo_config = "obj.names"
# convert_tfrecord_to_yolo(input_folder, output_folder, yolo_config)

input_folder = "../data/yolo/train"
output_folder = "../data/yolo-ann/train"
draw_bboxes_and_save(input_folder, output_folder)

def organize_files(input_path):
    # Paths for the images and labels folders
    images_path = os.path.join(input_path, 'images')
    labels_path = os.path.join(input_path, 'labels')

    # Create folders if they do not exist
    if not os.path.exists(images_path):
        os.makedirs(images_path)
    if not os.path.exists(labels_path):
        os.makedirs(labels_path)

    # Move jpg files to images folder and txt files to labels folder
    for filename in os.listdir(input_path):
        file_path = os.path.join(input_path, filename)
        if filename.endswith('.jpg'):
            # Move jpg files
            shutil.move(file_path, os.path.join(images_path, filename))
        elif filename.endswith('.txt'):
            # Move txt files
            shutil.move(file_path, os.path.join(labels_path, filename))

# Example usage:
input_dir = "../data/yolo/val"
#organize_files(input_dir)
