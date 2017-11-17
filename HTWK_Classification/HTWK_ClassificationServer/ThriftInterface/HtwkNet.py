import tensorflow as tf

config = tf.ConfigProto()
config.gpu_options.allow_growth = True
config.gpu_options.per_process_gpu_memory_fraction = 0.80
session = tf.Session(config=config)

from keras.backend.tensorflow_backend import set_session
set_session(session)

import cv2
from PIL import Image
import numpy as np
import keras
from keras.preprocessing.image import img_to_array
from keras.applications.vgg16 import preprocess_input as vgg16_preprocess_input
import htwk_keras_utils
import json
import sys


default_path = 'model/htwk_aadc_v3_nadam_vgg16_finetune.final.h5'
input_shape = (224, 224) # default input shape for VGG
pre_func = vgg16_preprocess_input # imagenet mean subtraction

# classes = ['car', 'sign', 'pylon', 'gabi', 'steffi', 'road']
# labelmap = {0: 'car', 1: 'child', 2: 'pylon', 3: 'road', 4: 'sign', 5: 'steffi'}


class HtwkNet:
    def __init__(self):
        self.model = None
        self.modelPath = None

    def load(self, path=None):
        # load the model from given path
        if path is None:
            path = default_path
        if self.modelPath == path:
            return
        if self.model is not None:
            del self.model
        sys.stdout.flush()
        self.model = keras.models.load_model(
                filepath=path,
                custom_objects={'f1': htwk_keras_utils.f1})
        self.modelPath = path

    def classify(self, image_data):
        # preprocess the image data
        image = self.preprocess_image(image_data)
        # run image data through the net and get prediction from softmax output tensor
        preds = self.model.predict(image)[0]
        return preds

    @staticmethod
    def preprocess_image(image_data):
        # create byte array from input string
        image = np.fromstring(
                string=image_data,
                dtype=np.uint8)
        # load array as cv mat
        image = cv2.imdecode(
                buf=image,
                flags=cv2.IMREAD_COLOR)
        # resize to input shape
        image = cv2.resize(
                src=image,
                dsize=input_shape,
                interpolation=cv2.INTER_NEAREST)
        # convert to RGB
        image = cv2.cvtColor(
                src=image,
                code=cv2.COLOR_BGR2RGB)
        # todo skip PIL and do everything in opencv, then convert to numpy array
        # todo find out why I cannot just use a Mat object directly. PIL magic?
        # convert to array
        image = Image.fromarray(image)
        image = img_to_array(image)
        # expand dims to inlcude batch size (of 1)
        image = np.expand_dims(image, axis=0)
        # subtract imagenet pixel mean
        image = pre_func(image)
        return image

    @staticmethod
    def get_label_map(labelmap=None):
        if labelmap is None:
            return {}
        with open('labelmap.json', 'r') as f:
            return json.load(f)
