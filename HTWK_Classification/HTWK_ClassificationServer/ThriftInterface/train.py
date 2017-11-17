import sys
import numpy as np
import keras
import json
import pickle
import time
import htwk_keras_utils
from keras import optimizers
from keras.layers import Dropout, Flatten, Dense, Conv2D, MaxPooling2D
from keras.models import Model
from keras.callbacks import CSVLogger
from keras.applications import Xception, VGG16, MobileNet
from keras.preprocessing.image import array_to_img, img_to_array, load_img
from keras.preprocessing.image import ImageDataGenerator
from keras.applications.xception import preprocess_input as xception_preprocess_input
from keras.applications.vgg16 import preprocess_input as vgg16_preprocess_input
from keras.applications.mobilenet import preprocess_input as mobilenet_preprocess_input

train_data_path= 'datasets/htwk_aadc_v4/train'
val_data_path = 'datasets/htwk_aadc_v4/val'
model_out_path = 'models/'
model_in_path = 'models/htwk_aadc_v4_vgg16.final.h5'
arch = 'vgg16'
net_name = 'htwk_aadc_v4_2_'
# 16:? ~240ms + ~250ms
# 32:
#   vgg16 (flatten fc512 drop.5 fc512 logit)
#       205ms transfer + 225ms finetune
# 64: (too large for 6gb vram)
batch_size = 32
epochs = 100 # n transfer + n fine tune
do_transfer = True
sizes = {'vgg16':224,'xception':299,'mobilenet':224}
freeze_layers = {'vgg16':4,'xception':17,'mobilenet':13}
transfer_layers = {'vgg16':0,'xception':0,'mobilenet':7}

def loadModel(model, im_size):
    return model(
            weights = 'imagenet',
            include_top = False,
            input_shape = (im_size, im_size, 3)) # pooling = 'avg')

def loadModelFile(path):
    return keras.models.load_model(
            path,
            custom_objects = {'f1': htwk_keras_utils.f1})

def prepareDataGenerator(train_path, val_path, preprocess_func, im_size, batch_size):
    train_datagen = ImageDataGenerator(
            rotation_range = 15,
            width_shift_range = 0.2,
            height_shift_range = 0.2,
            rescale = 1./255.,
            shear_range = 0.1,
            zoom_range = 0.2,
            horizontal_flip = True,
            fill_mode = 'nearest',
            preprocessing_function = preprocess_func,
            data_format = 'channels_last')

    val_datagen = ImageDataGenerator(
            rescale=1./255.,
            preprocessing_function = preprocess_func,
            data_format = 'channels_last')

    train_generator = train_datagen.flow_from_directory(
            train_path,
            target_size = (im_size, im_size),
            batch_size = batch_size,
            class_mode = 'categorical')

    val_generator = val_datagen.flow_from_directory(
            val_path,
            target_size = (im_size, im_size),
            batch_size = batch_size,
            class_mode = 'categorical')

    return train_generator, val_generator

def lrScheduleFinetune(epochIndex):
    if epochIndex > 65:
        newLr = 0.00001
    else:
        newLr = 0.00010
    print("scheduler: " + str(epochIndex) + " : " + str(newLr))
    return newLr
    
def lrScheduleTransfer(epochIndex):
    if epochIndex > 65:
        newLr = 0.00100
    else:
        newLr = 0.01000
    print("scheduler: " + str(epochIndex) + " : " + str(newLr))
    return newLr
    
plateauLr = keras.callbacks.ReduceLROnPlateau(
        monitor='loss',
        factor=0.5,
        patience=5,
        verbose=1,
        mode='auto',
        epsilon=0.0001,
        cooldown=0,
        min_lr=0.000001)

start_time = time.time()

net_out_path = model_out_path + net_name + arch

models = {'vgg16':VGG16,'xception':Xception,'mobilenet':MobileNet}
ppfuncs = {'vgg16':vgg16_preprocess_input,'xception':xception_preprocess_input, 'mobilenet':mobilenet_preprocess_input}


im_size = sizes[arch]
preprocess_func = ppfuncs[arch]
finetune_num = freeze_layers[arch]
transfer_num = transfer_layers[arch]


print('==== [INFO] loading model ' + arch)
if do_transfer:
    model = loadModel(model = models[arch], im_size = im_size)
else:
    model = loadModelFile(path = model_in_path)
print('==== [INFO] ' + arch + ' model loaded')

print('==== [INFO] preparing data generators')
train_generator, val_generator = prepareDataGenerator(
    train_path = train_data_path,
    val_path = val_data_path,
    preprocess_func = preprocess_func,
    im_size = im_size,
    batch_size = batch_size)

nb_val = val_generator.samples
nb_train = train_generator.samples
nb_class = train_generator.num_classes

print('==== [INFO] saving labelmap')
labelmap_json = json.dumps(train_generator.class_indices)
with open(net_out_path + '_labelmap.json', 'w') as f:
    f.write(labelmap_json)

print('==== [INFO] preparing top')
only_finetune_layers = 5
if do_transfer:
    model_num = len(model.layers)
    x = model.output
#    x = model.get_layer('fc2').output
#    x = Conv2D(1024, (3, 3), activation='relu', padding='same', name='Ex_block5_conv1', kernel_initializer='glorot_uniform')(x)
#    x = Conv2D(1024, (3, 3), activation='relu', padding='same', name='Ex_block5_conv2', kernel_initializer='glorot_uniform')(x)
#    x = Conv2D(1024, (3, 3), activation='relu', padding='same', name='Ex_block5_conv3', kernel_initializer='glorot_uniform')(x)
#    x = MaxPooling2D((2, 2), strides=(2, 2), name='Ex_block5_pool')(x)

    x = Flatten(name='flattenEx')(x)
    x = Dense(
            units = 512,
            name = 'fc1Ex',
            activation = 'sigmoid')(x)
    x = Dropout(0.5, name='dropout1Ex')(x)
    x = Dense(
            units = 512,
            name = 'fc2Ex',
            activation = 'sigmoid')(x)
    logit = Dense(
            units = nb_class,
            name = 'logit',
            activation = 'softmax')(x)
    final_model = Model(input = model.input, output = logit)
    extra_num = len(final_model.layers) - model_num
else:
    final_model = model
    extra_num = only_finetune_layers

if do_transfer:
    print('==== [INFO] freezing layers')
    print('==== [INFO] ---- freeze')
    for layer in final_model.layers:
        print(layer.name)
        layer.trainable = False
    print('==== [INFO] ---- unfreeze')
    for layer in final_model.layers[len(final_model.layers) - (extra_num + transfer_num):]:
        print(layer.name)
        layer.trainable = True

    print('==== [INFO] compiling model')
    final_model.compile(
            loss = 'categorical_crossentropy',
#        optimizer = optimizers.SGD(
#            lr = 1e-2,
            #decay = 1e-6,
#            momentum = 0.9,
#            nesterov = True),
#            optimizer='nadam',
            optimizer = 'rmsprop',
            metrics = ['accuracy', htwk_keras_utils.f1])
    final_model.summary()

    print('==== [INFO] saving model')
    final_model.save(net_out_path + '_transfer.init.h5')

    print('==== [INFO] training ...')
    checkpointCallback = keras.callbacks.ModelCheckpoint(
            net_out_path + '_transfer.e{epoch:03d}-l{val_loss:.4f}-a{val_acc:.4f}.h5',
            monitor = 'val_acc',
            verbose = 1,
            save_best_only = True,
            save_weights_only = False,
            mode = 'auto',
            period = 1)

    scheduler = keras.callbacks.LearningRateScheduler(lrScheduleTransfer)

    csv_logger = CSVLogger(net_out_path + '_transfer.csv')
    history_transfer = final_model.fit_generator(
            train_generator,
            steps_per_epoch = nb_train // batch_size,
            epochs = epochs,
            validation_data = val_generator,
            validation_steps = nb_val // batch_size,
            callbacks = [checkpointCallback, csv_logger, plateauLr])

    print('==== [INFO] saving model')
    final_model.save(net_out_path + '_transfer.final.h5')

    print('==== [INFO] ==== done ====')


print('==== [INFO] FINETUNING')


print('==== [INFO] freezing layers')
print('==== [INFO] ---- freeze')
for layer in final_model.layers:
    print(layer.name)
    layer.trainable = False
print('==== [INFO] ---- unfreeze')
for layer in final_model.layers[len(final_model.layers) - (extra_num + finetune_num):]:
    print(layer.name)
    layer.trainable = True

print('==== [INFO] compiling model')
final_model.compile(
        loss = 'categorical_crossentropy',
        optimizer = optimizers.SGD(
            lr = 1e-4,
            #decay = 1e-6,
            momentum = 0.9,
            nesterov = True
        ),
        metrics = ['accuracy', htwk_keras_utils.f1])
final_model.summary()

print('==== [INFO] saving model')
final_model.save(net_out_path + '_finetune.init.h5')

print('==== [INFO] training ...')
checkpointCallback = keras.callbacks.ModelCheckpoint(
        net_out_path + '_finetune.e{epoch:03d}-l{val_loss:.4f}-a{val_acc:.4f}.h5',
        monitor = 'val_acc',
        verbose = 1,
        save_best_only = False,
        save_weights_only = False,
        mode = 'auto',
        period = 1)

scheduler = keras.callbacks.LearningRateScheduler(lrScheduleFinetune)

csv_logger = CSVLogger(net_out_path + '_finetune.csv')
history_finetune = final_model.fit_generator(
        train_generator,
        steps_per_epoch = nb_train // batch_size,
        epochs = epochs,
        validation_data = val_generator,
        validation_steps = nb_val // batch_size,
        callbacks = [checkpointCallback, csv_logger, plateauLr])

print('==== [INFO] saving model')
final_model.save(net_out_path + '_finetune.final.h5')

print('==== [INFO] ==== done ====')

end_time = time.time()
duration = end_time - start_time
print('==== [INFO] took ' + str(duration) + ' seconds')

#    samplewise_center=False,
#    featurewise_std_normalization=False,
#    samplewise_std_normalization=False,
#    zca_whitening=False,
#    zca_epsilon=1e-6,
#    rotation_range=0.,
#    width_shift_range=0.,
#    height_shift_range=0.,
#    shear_range=0.,
#    zoom_range=0.,
#    channel_shift_range=0.,
#    fill_mode='nearest',
#    cval=0.,
#    horizontal_flip=False,
#    vertical_flip=False,
#    rescale=None,

