import numpy as np
import argparse
import cv2
import keras
import sys
import os
from keras.applications import MobileNet
from keras.applications.mobilenet import relu6, DepthwiseConv2D
from keras.applications import imagenet_utils
from keras.preprocessing.image import img_to_array
from keras.preprocessing.image import load_img
from keras.applications.xception import preprocess_input as xception_preprocess_input
from keras.applications.vgg16 import preprocess_input as vgg16_preprocess_input
from keras.applications.mobilenet import preprocess_input as mobilenet_preprocess_input
import htwk_keras_utils

# vgg16_1:  'acc': 0.7634 84280357649, 'f1': 0.5183 08204425306     0
#
#   car:    'acc': 0.9661 01694915252, 'f1': 0.9047 61904761904
#   sign:   'acc': 0.5552 69922879178, 'f1': 0.2938 77551020408
#   pylon:  'acc': 0.9932 88590604025, 'f1': 0.9801 32450331126
#   gabi:   'acc': 0.3715 84699453551, 'f1': 0.1646 48910411622
#   steffi: 'acc': 0.9421 12879884222, 'f1': 0.8443 57976653697
#   road:   'acc': 0.7518 84852638801, 'f1': 0.5025 19468621161
# 

# vgg16_2:  'acc': 0.8413 61407556970, 'f1': 0.6387 12502737037     1
#
#   car:    'acc': 0.7288 13559322036, 'f1': 0.4725 27472527471
#   sign:   'acc': 0.9794 34447300769, 'f1': 0.9407 40740740740
#   pylon:  'acc': 0.9194 63087248321, 'f1': 0.7919 07514450867
#   gabi:   'acc': 0.5546 44808743170, 'f1': 0.2933 52601156069
#   steffi: 'acc': 0.9956 58465991315, 'f1': 0.9870 87517934003 <-
#   road:   'acc': 0.8272 78958190551, 'f1': 0.6148 75191034127

# vgg16_3:  'acc': 0.9348 13960196152, 'f1': 0.8269 96682827256 <-  5
#
#   car:    'acc': 0.9903 14769975784, 'f1': 0.9714 96437054632 <-
#   sign:   'acc': 0.9897 17223650383, 'f1': 0.9697 73299748110 <-
#   pylon:  'acc': 0.9932 88590604025, 'f1': 0.9801 32450331126 <-
#   gabi:   'acc': 0.8497 26775956286, 'f1': 0.6533 61344537814 <-
#   steffi: 'acc': 0.9826 33863965265, 'f1': 0.9496 50349650350
#   road:   'acc': 0.8971 89856065804, 'f1': 0.7441 72825469013 <-

# vgg16_4:  'acc': 0.6945 48601096031, 'f1': 0.4311 54879140559 meh

#----
# V3
#----

# v3vgg16_1:'acc': 0.9324 09381663153, 'f1': 0.8213 74906085644     2
#
#   car:    'acc': 0.7746 21212121210, 'f1': 0.5339 42558746735
#   sign:   'acc': 0.9922 58064516132, 'f1': 0.9771 28335451080 <-
#   pylon:  'acc': 0.9721 25435540072, 'f1': 0.9207 92079207920
#   gabi:   'acc': 0.7789 69957081544, 'f1': 0.5401 78571428570
#   steffi: 'acc': 0.9642 38410596027, 'f1': 0.8998 76390605687
#   road:   'acc': 0.9712 61309207010, 'f1': 0.9184 70055359838 <-

# vgg16_5:  'acc': 0.7616 20469083176, 'f1': 0.5157 37799595725 meh

# vgg16_3:  'acc': 0.9187 63326226051, 'f1': 0.7903 52164343355

# v3vgg16_2 nadam:
# all:	    'acc': 0.8837 95309168,     'f1': 0.7171 28027682
# car:	    'acc': 0.9545 45454545,     'f1': 0.8750 0
# sign:	    'acc': 0.9754 83870968,     'f1': 0.9298 89298893
# pylon:	'acc': 0.9930 31358885,     'f1': 0.9793 81443299
# gabi:	    'acc': 0.8154 50643777,     'f1': 0.5956 11285266
# steffi:	'acc': 0.9774 83443709,     'f1': 0.9353 6121673
# road:	    'acc': 0.7887 17402874,     'f1': 0.5544 332211

# v3vgg16_3:                                                        2
# all:	    'acc': 0.7057 56929638,     'f1': 0.4442 95302013
# car:	    'acc': 0.9715 90909091,     'f1': 0.9193 5483871
# sign: 	'acc': 0.9574 19354839,     'f1': 0.8822 82996433
# pylon:    'acc': 1.0000 00000000,                             <-
# gabi:     'acc': 0.8605 15021459,                             <-
# steffi:   'acc': 0.9629 13907284,
# road:     'acc': 0.3406 06705694,
# div0 everything was fine, but road only: 0.35 :|
 
# UP NEXT: keep top of vgg16, only do finetuning on fc1, fc2, and logit

model_load = 'model/htwk_aadc_v3_5_sgd_plateau_vgg16_finetune.final.h5'
input_shape = (224, 224)
pre_func = vgg16_preprocess_input

test_class = 'car'
classes = ['car', 'sign', 'pylon', 'gabi', 'steffi', 'road']

print("[INFO] loading model")
model = keras.models.load_model(
        model_load,
        custom_objects={'f1': htwk_keras_utils.f1, 'relu6':relu6, 'DepthwiseConv2D':DepthwiseConv2D})

ratio = 0.0
hit = 0.0
miss = 0.0
count = 0.0

tp = 0.0
tn = 0.0
fp = 0.0
fn = 0.0
tot = 0.0

def calc_conf(tp,tn,fp,fn,tot):
    cond_pos = tp + fn      # condition positive
    cond_neg = fp + tn      # condition negative
    pred_pos = tp + fp      # predicted condition positive
    pred_neg = fn + tn      # predicted condition negative
    tp_r = tp / cond_pos    # recall
    fn_r = fn / cond_pos    # miss rate
    fp_r = fp / cond_neg    # fall-out
    tn_r = tn / cond_neg    # specificity
    preval = cond_pos / tot # prevalence
    acc = (tp + tn) / tot   # accuracy
    pp_v = tp / pred_pos    # precision
    fo_r = fn / pred_neg    # false omission rate
    fd_r = fp / pred_pos    # false discovery rate
    np_v = tn / pred_neg    # negative predictive value
    lr_p = tp_r / fp_r      # positive likelihood ratio
    lr_n = fn_r / tn_r      # negative likelihood ratio
    do_r = lr_p / lr_n      # diagnostics odds ratio
    f1 = 2.0 / ((1.0 / tp_r) + (1.0 / pp_v)) # F1 score
    return {
            'cond_pos':cond_pos,
            'cond_neg':cond_neg,
            'pred_pos':pred_pos,
            'pred_neg':pred_neg,
            'tp_r':tp_r,
            'fn_r':fn_r,
            'fp_r':fp_r,
            'tn_r':tn_r,
            'preval':preval,
            'acc':acc,
            'pp_v':pp_v,
            'fo_r':fo_r,
            'fd_r':fd_r,
            'np_v':np_v,
            'lr_p':lr_p,
            'lr_n':lr_n,
            'do_r':do_r,
            'f1':f1
    }

vals = {
        'car':{'c':0,'h':0,'m':0,'r':0,'tp':0,'tn':0,'fp':0,'fn':0,'tot':0},
        'sign':{'c':0,'h':0,'m':0,'r':0,'tp':0,'tn':0,'fp':0,'fn':0,'tot':0},
        'pylon':{'c':0,'h':0,'m':0,'r':0,'tp':0,'tn':0,'fp':0,'fn':0,'tot':0},
        'gabi':{'c':0,'h':0,'m':0,'r':0,'tp':0,'tn':0,'fp':0,'fn':0,'tot':0},
        'steffi':{'c':0,'h':0,'m':0,'r':0,'tp':0,'tn':0,'fp':0,'fn':0,'tot':0},
        'road':{'c':0,'h':0,'m':0,'r':0,'tp':0,'tn':0,'fp':0,'fn':0,'tot':0}
}

for cl in classes:
    test_dir = 'AADC/datasets/htwk_aadc_v3/val/' + cl + '/'
    for filename in os.listdir(test_dir):
        if filename.endswith(".png") or filename.endswith(".tif"): 
            #print("[INFO] loading and pre-processing image")
            image = load_img(test_dir + filename, target_size=input_shape)
            image = img_to_array(image)

            image = np.expand_dims(image, axis=0)
            image = pre_func(image)
            #print(image)
            #print("[INFO] predicting")
            preds = model.predict(image)[0]

            #print(preds)
            preds_dec = {'car':preds[0], 'sign':preds[4], 'pylon':preds[2], 'gabi':preds[1], 'steffi':preds[5], 'road':preds[3]}
            
            top1str = ''
            top1scr = 0
            for m in preds_dec:
                if preds_dec[m] > top1scr:
                    top1scr = preds_dec[m]
                    top1str = m
                
            if top1str == cl:
                hit += 1.0
                vals[cl]['h'] += 1.0
                tp += 1.0/6.0
                vals[cl]['tp'] += 1.0/6.0
                tn += 5.0/6.0
                vals[cl]['tn'] += 5.0/6.0
            else:
                miss += 1.0
                vals[cl]['m'] += 1.0
                fp += 5.0/6.0
                vals[cl]['fp'] += 5.0/6.0
                fn += 1.0/6.0
                vals[cl]['fn'] += 1.0/6.0
            
            tot += 1.0
            vals[cl]['tot'] += 1.0
            count += 1.0
            vals[cl]['c'] += 1.0
            
            ratio = hit / count
            vals[cl]['r'] = vals[cl]['h'] / vals[cl]['c']        
                    
            if count % 100 == 0:
                print("----")
                print(ratio)
                print(cl)
                print(vals[cl])

    print(cl + " :: ")
    print(vals[cl])

print("========")
print("final: ")
print("")
print(vals)
print("")
print("========")
print("ratio: " + str(ratio))
print("count: " + str(count))
print("hit: " + str(hit))
print("miss: " + str(miss))

print("====")
matrix = calc_conf(tp = tp, tn = tn, fp = fp, fn = fn, tot = tot)
print(matrix)
print("====")

print("all:\t'acc': " + str(matrix['acc']) + ", 'f1': " + str(matrix['f1']))

for cl in classes:
    matrix = calc_conf(
        tp = vals[cl]['tp'],
        tn = vals[cl]['tn'],
        fp = vals[cl]['fp'],
        fn = vals[cl]['fn'],
        tot = vals[cl]['tot']
    ) # tp,tn,fp,fn,tot
    print(cl + ":\t'acc': " + str(matrix['acc']) + ", 'f1': " + str(matrix['f1']))
    
#{'car': 0, 'sign': 4, 'pylon': 2, 'gabi': 1, 'steffi': 5, 'road': 3}

