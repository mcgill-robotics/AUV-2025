import os, shutil
from os import listdir
from os.path import isfile, join
import cv2
import albumentations as A
import copy
import random

#Authors
#Antoine Dangeard
#Elie-Dimitri Abdo

#given a list of samples, make two copies of each sample that are darker/brighter to simulate differently lit environments
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
        alpha = 0.3 # alpha is contrast, [0,1] to lower contrast and > 1 to higher 
        beta = 0 
        decontrasted_img = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        out.append((decontrasted_img, label))
    samples.extend(out)
    return samples
    
#given a list of samples, make a copy of each sample but with camera noise added to the image to simulate different camera feeds
def noiseAugment(samples):
    out = []
    for sample in samples:
        image, label = sample
        transform = A.Compose([ A.augmentations.transforms.ISONoise(color_shift=(0.1, 0.1), intensity=(0.8, 0.8), always_apply=True) ])
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

#code adapted from packtpub.com
def create_LUT_shift(x, y): return UnivariateSpline(x, y)(range(256))

#increase intensity of blues in given image
def make_bluer(img, color_shift_intensity):
    img_b, img_g, img_r = cv2.split(img) #split by channel
    img_b += color_shift_intensity
    img_b.clip(0,255)
    img = cv2.merge((img_b, img_g, img_r)) #merge adjusted channels
    return img

#increase intensity of greens in given image
def make_greener(img, color_shift_intensity):
    img_b, img_g, img_r = cv2.split(img) #split by channel
    img_g += color_shift_intensity
    img_g.clip(0,255)
    img = cv2.merge((img_b, img_g, img_r)) #merge adjusted channels
    return img

#given a list of samples, make two copies of each sample (one bluer, one greener) to simulate different pools + color attenuation
def colorAugment(samples):
    out = []
    color_shift_intensity = int(255*0.25)
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
out_folder = os.path.realpath(os.path.dirname(__file__)) + "/clean-data"
in_folder = os.path.realpath(os.path.dirname(__file__)) + "/raw-datasets"

if __name__ == '__main__':
    print("Loading data...")
    data = loadInputData(in_folder)

    #uncomment for visualizing individual augmentations
    #visualizeAugmentation(data[0][0], colorAugment)
    #exit()

    #order of augmentations should be optimized so that slower augmentations are done first
    print("Augmenting color...")
    data = colorAugment(data) # times 3
    print("Augmenting noise...")
    data = noiseAugment(data) # times 2
    print("Augmenting resolution...")
    data = resolutionAugment(data) # times 2
    print("Augmenting contrast...")
    data = contrastAugment(data) # times 2
    print("Augmenting brightness...")
    data = brightnessAugment(data) # times 3
    print("Augmenting blur...")
    data = blurAugment(data) # times 2
    #overall times 144
    print("Splitting data...")
    train, test, val = splitData(data, train_test_val_split)
    print("Exporting data...")
    sendToFolders(train, out_folder + "/train")
    sendToFolders(test, out_folder + "/test")
    sendToFolders(val, out_folder + "/val")
    print("Done.")