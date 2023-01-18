import os
from os import listdir
from os.path import isfile, join
import cv2

def brightnessAugment(samples):
    out = []
    for sample in samples:
        image, label = sample
        beta = 80
        beta1 = -50
        alpha = 1 
        bright_img = cv2.convertScaleAbs(image, alpha=alpha, beta=beta) #beta is brightness [-127;127]
        dark_img = cv2.convertScaleAbs(image, alpha=alpha, beta=beta1)
        out.append((bright_img, label))
        out.append((dark_img, label))
    samples.extend(out)
    return samples

def blurAugment(samples):
    out = []
    for sample in samples:
        image, label = sample
        ksize = (20, 20) # lower to lower blur  
        blurred_img = cv2.blur(image, ksize) 
        out.append((blurred_img, label))
    samples.extend(out)
    return samples

def contrastAugment(samples):
    out = []
    for sample in samples:
        image, label = sample
        alpha = 0.3 # alpha is contrast, [0,1] to lower contrast and > 1 to higher 
        beta = 0 
        decontrasted_img = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        out.append((decontrasted_img, label))
    samples.extend(out)
    return samples
    
def noiseAugment(samples):
    out = []
    for sample in samples:
        image, label = sample
        noisy_img = add_noise(image)
        out.append((noisy_img, label))
    samples.extend(out)
    return samples

def colorAugment(samples):
    out = []
    for sample in samples:
        image, label = sample
        blue_img = change_color_balance(image, 'b')
        green_img = change_color_balance(image, 'g')
        out.append((blue_img, label))
        out.append((green_img, label))
    samples.extend(out)
    return samples

def sendToFolders(samples, data_folder):
    for sample in samples:
        image, label = sample
        cv2.imwrite('out.png', image)

def splitData(samples, splits):
    pass

def testAugmentation(img, aug):
    cv2.imshow('og', img)
    cv2.waitKey(0)
    print("test" + str(aug([(img, "")])))
    for augmented in aug([(img, "")])[1:]:
        cv2.imshow('augmented',augmented[0])
        cv2.waitKey(0)

def loadInputData(source_folder):
    samples = []
    img_filenames = [f for f in listdir(source_folder + '/images') if isfile(join(source_folder + '/images', f))]
    for img_filename in img_filenames:
        label_filename = os.path.splitext(img_filename)[0] + ".txt"
        with open(source_folder + "/labels/" + label_filename) as f:
            bounding_box = f.read()
            bounding_box = bounding_box.split(" ")
        img = cv2.imread(source_folder + '/images/' + img_filename)
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
    testAugmentation(data[5][0], contrastAugment)
    #augmented = brightnessAugment(data) # times 3
    #augmented = blurAugment(augmented) # times 2
    #augmented = contrastAugment(augmented) # times 2
    #augmented = noiseAugment(augmented) # times 2
    #augmented = qualityAugment() # times 2
    #final = colorAugment(augmented) # times 3
    #train, test, val = splitData(final, train_test_val_split)
    #sendToFolders(train, out_folder + "/train")
    #sendToFolders(test, out_folder + "/test")
    #sendToFolders(val, out_folder + "/val")


    #USE ALBUMENTATIONS!!


