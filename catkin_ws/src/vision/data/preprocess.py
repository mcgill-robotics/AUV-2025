import os
from os import listdir
from os.path import isfile, join
import cv2

def brightnessAugment(samples):
    out = samples
    for sample in samples:
        image, label = sample
        beta = 120 
        beta1 = -120
        alpha = 1 
        bright_img = cv2.convertScaleAbs(image, alpha=alpha, beta=beta) #beta is brightness [-127;127]
        dark_img = cv2.convertScaleAbs(image, alpha=alpha, beta=beta1)
        out.append((bright_img, label))
        out.append((dark_img, label))
    return out

def blurAugment(samples):
    out = samples
    for sample in samples:
        image, label = sample
        ksize = (10, 10) # lower to lower blur  
        blurred_img = cv2.blur(image, ksize) 
        out.append((blurred_img, label))
    return out

def deContrast(samples):
    out = samples
    for sample in samples:
        image, label = sample
        alpha = 0.5 # alpha is contrast, [0,1] to lower contrast and >1 to higher 
        beta = 0 
        decontrasted_img = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        out.append((decontrasted_img, label))
    return out
    
def noiseAugment(samples):
    out = samples
    for sample in samples:
        image, label = sample
        noisy_img = add_noise(image)
        out.append((noisy_img, label))
    return out

def colorAugment(samples):
    out = samples
    for sample in samples:
        image, label = sample
        blue_img = change_color_balance(image, 'b')
        green_img = change_color_balance(image, 'g')
        out.append((blue_img, label))
        out.append((green_img, label))
    return out

def sendToFolders(samples, data_folder):
    cv2.imwrite('out.png', image)

def splitData(samples, splits):
    pass

def testAugmentation(img, aug):
    cv2.imshow('og', img)
    augmented_img = aug(img)
    cv2.imshow('augmented',augmented_img)

def loadInputData(source_folder):
    samples = []
    img_filenames = [f for f in listdir(source_folder + '/images') if isfile(join(source_folder + '/images', f))]
    for img_filename in img_filenames:
        label_filename = os.path.splitext(img_filename)[0] + ".txt"
        with open(source_folder + "/labels/" + label_filename) as f:
            bounding_box = f.read()
            bounding_box = bounding_box.split(" ")
        img = cv2.imread(img_filename)
        samples.append( (img, bounding_box) )
        #label = loadBoundingBox(img_file)
    return samples

img_size = (416, 416)

train_test_val_split = (0.7, 0.2, 0.1)

out_folder = os.path.realpath(os.path.dirname(__file__)) + "/clean-data"

in_folder = os.path.realpath(os.path.dirname(__file__)) + "/raw-datasets"

if __name__ == '__main__':
    #ensure there is an argument
    data = loadInputData(in_folder)
    augmented = brightnessAugment(data) # times 3
    augmented = blurAugment(augmented) # times 2
    augmented = deContrast(augmented) # times 2
    augmented = noiseAugment(augmented) # times 2
    final = colorAugment(augmented) # times 3
    train, test, val = splitData(final, train_test_val_split)
    sendToFolders(train, out_folder + "/train")
    sendToFolders(test, out_folder + "/test")
    sendToFolders(val, out_folder + "/val")


    #USE ALBUMENTATIONS!!


