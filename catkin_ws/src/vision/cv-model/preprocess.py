import os, shutil
from os import listdir
from os.path import isfile, join
import cv2
import albumentations as A
import copy
import random
import numpy as np

#Authors
#Antoine Dangeard
#Elie-Dimitri Abdo

#given a list of samples, make two copies of each sample that are darker/brighter to simulate differently lit environments
def brightnessAugment(samples):
    out = []
    for sample in samples:
        image, label = sample
        transform = A.Compose([ A.augmentations.transforms.ColorJitter (brightness=(1.5, 1.5), contrast=0, saturation=0, hue=0, always_apply=True) ])
        bright_img = transform(image=image)["image"]
        transform = A.Compose([ A.augmentations.transforms.ColorJitter (brightness=(0.5, 0.5), contrast=0, saturation=0, hue=0, always_apply=True) ])
        dark_img = transform(image=image)["image"]
        out.append((bright_img, label))
        out.append((dark_img, label))
    samples.extend(out)
    return samples

#given a list of samples, make a copy of each sample but more blurred to simulate objects out of focus, dirty lenses, and backscattering
def blurAugment(samples):
    out = []
    for sample in samples:
        image, label = sample
        ksize = (20, 20) # lower to lower blur  
        blurred_img = cv2.blur(image, ksize) 
        out.append((blurred_img, label))
    samples.extend(out)
    return samples

#given a list of samples, make a copy of each sample but with a lower contrast image to simulate backscattering and over/under-exposure
def contrastAugment(samples):
    out = []
    for sample in samples:
        image, label = sample
        transform = A.Compose([ A.augmentations.transforms.ColorJitter (brightness=0, contrast=(0.3, 0.3), saturation=0, hue=0, always_apply=True) ])
        decontrasted_img = transform(image=image)["image"]
        out.append((decontrasted_img, label))
    samples.extend(out)
    return samples
    
#given a list of samples, make a copy of each sample but with camera noise added to the image to simulate different camera feeds
def noiseAugment(samples):
    out = []
    for sample in samples:
        image, label = sample
        transform = A.Compose([ A.augmentations.transforms.ISONoise(color_shift=(0.01, 0.01), intensity=(0.8, 0.8), always_apply=True) ])
        noisy_img = transform(image=image)["image"]
        out.append((noisy_img, label))
    samples.extend(out)
    return samples

#given a list of samples, make a copy of each sample but with the image downscaled (lower resolution of image) to simulate lower quality cameras/images
def resolutionAugment(samples):
    out = []
    for sample in samples:
        image, label = sample
        transform = A.Compose([ A.augmentations.transforms.Downscale(scale_min=0.25, scale_max=0.25, interpolation=A.augmentations.transforms.Downscale.Interpolation(downscale=cv2.INTER_NEAREST, upscale=cv2.INTER_NEAREST), always_apply=True) ])
        low_res_img = transform(image=image)["image"]
        out.append((low_res_img, label))
    samples.extend(out)
    return samples

#increase intensity of blues in given image
def make_bluer(img, color_shift_intensity):
    img_b, img_g, img_r = cv2.split(img) #split by channel
    img_b = np.uint16(img_b)
    img_b += color_shift_intensity
    np.clip(img_b, 0, 255, out=img_b)
    img_b = np.uint8(img_b)
    img = cv2.merge((img_b, img_g, img_r)) #merge adjusted channels
    return img

#increase intensity of greens in given image
def make_greener(img, color_shift_intensity):
    img_b, img_g, img_r = cv2.split(img) #split by channel
    img_g = np.uint16(img_g)
    img_g += color_shift_intensity
    np.clip(img_g, 0, 255, out=img_g)
    img_g = np.uint8(img_g)
    img = cv2.merge((img_b, img_g, img_r)) #merge adjusted channels
    return img

#given a list of samples, make two copies of each sample (one bluer, one greener) to simulate different pools + color attenuation
def colorAugment(samples):
    out = []
    color_shift_intensity = int(255*0.1)
    for sample in samples:
        image, label = sample
        blue_img = make_bluer(image, color_shift_intensity)
        green_img = make_greener(image, color_shift_intensity)
        out.append((blue_img, label))
        out.append((green_img, label))
    samples.extend(out)
    return samples

#remove all files/folders in folder
def clearFolder(folder):
    #get all directory/filenames in folder
    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                #delete all files
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                #recursively delete everything in sub-folders
                shutil.rmtree(file_path)
        except Exception as e:
            print('Failed to delete %s. Reason: %s' % (file_path, e))

#output given samples to given folder so that each image has corresponding label with same filename in /images and /labels subfolders respectively
def sendToFolders(samples, output_folder):
    #remove all files/folders in output folders
    clearFolder(output_folder + "/images")
    clearFolder(output_folder + "/labels")
    instance_count = 0 #keep track of instance count for filename
    for sample in samples:
        image, label = sample
        #write image to file in /images
        cv2.imwrite(output_folder + '/images/img' + str(instance_count) + '.png', image)
        #write label to file in /labels
        with open(output_folder + '/labels/img' + str(instance_count) + '.txt', 'w+') as f:
            #each box gets its own line
            for box in label:
                f.write(box)
        instance_count += 1

#given array of image/label arrays, and an integer of how to split the data, returns the dataset split accordingly
def splitData(samples, splits):
    #for now we just shuffle the data to hopefully get a similar similar sample size
    # of images for every class in the train, test and val splits
    #ideally in the future we should split by class and then combine together to make sure the datasets are balanced

    #shuffle data
    random.shuffle(samples)
    #get indices for split
    splits = [int(len(samples)*s) for s in splits]
    #return split data
    return samples[:splits[0]], samples[splits[0]:splits[0]+splits[1]], samples[splits[0]+splits[1]:]

#given a single image and augmentation function, displays the image before and images after augmentation
def visualizeAugmentation(img, aug):
    #show original image
    cv2.imshow('og', img)
    cv2.waitKey(0)
    #show all augmented images
    for augmented in aug([(img, "")])[1:]:
        cv2.imshow('augmented',augmented[0])
        cv2.waitKey(0)

#given source dataset folder, loads all images and labels into arrays
def loadInputData(source_folder):
    samples = []
    #get all filenames in the /images subfolder of given source_folder
    img_filenames = [f for f in listdir(source_folder + '/images') if isfile(join(source_folder + '/images', f))]
    for img_filename in img_filenames:
        #load image at that filename
        img = cv2.imread(source_folder + '/images/' + img_filename)
        #got label filename corresponding to the image
        label_filename = os.path.splitext(img_filename)[0] + ".txt"
        #load in the label file contents
        with open(source_folder + "/labels/" + label_filename) as f:
            #build array of bounding boxes (each line its own element)
            bounding_boxes = []
            for line in f.read().split("\n"):
                bounding_boxes.append(line)
        #add image and label to sample set
        samples.append( (img, bounding_boxes) )
    return samples

train_test_val_split = (0.7, 0.2, 0.1)
pwd = os.path.realpath(os.path.dirname(__file__))
out_folder = pwd + "/data/augmented"
in_folder = pwd + "/data/raw"

if __name__ == '__main__':
    print("Loading data...")
    data = loadInputData(in_folder)

    #uncomment for visualizing individual augmentations
    #visualizeAugmentation(data[0][0], colorAugment)
    #exit()

    #order of augmentations should be optimized so that slower augmentations are done first
    print("Augmenting noise...")
    data = noiseAugment(data) # times 2
    print("Augmenting resolution...")
    data = resolutionAugment(data) # times 2
    print("Augmenting color...")
    data = colorAugment(data) # times 3
    print("Augmenting brightness...")
    data = brightnessAugment(data) # times 3
    print("Augmenting contrast...")
    data = contrastAugment(data) # times 2
    print("Augmenting blur...")
    data = blurAugment(data) # times 2
    print("Splitting data...")
    train, test, val = splitData(data, train_test_val_split)
    print("Exporting training data...")
    sendToFolders(train, out_folder + "/train")
    print("Exporting test data...")
    sendToFolders(test, out_folder + "/test")
    print("Exporting val data...")
    sendToFolders(val, out_folder + "/val")
    print("Done.")