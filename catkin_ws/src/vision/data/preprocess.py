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

def padToSize(samples, target_size):
    out = []
    for sample in samples:
        image, label = sample
        resized_img, resized_label = resize(image, label, target_size)
        padded_img, padded_label = pad(resized_img, resized_label, target_size)
        out.append((padded_img, padded_label))
    return out

def sendToFolders(samples, data_folder):

def splitData(samples, splits):

def loadInputData(source_folder):

    return samples

img_size = (416, 416)

train_test_val_split = (0.7, 0.2, 0.1)

out_folder = "clean-data"

if __name__ == '__main__':
    #ensure there is an argument
    data = loadInputData(cli_arg)
    #not sure about all of these, may result in too many samples
    augmented = brightnessAugment(augmented) # times 3
    augmented = blurAugment(augmented) # times 2
    augmented = deContrast(augmented) # times 2
    augmented = noiseAugment(augmented) # times 2
    augmented = colorAugment(augmented) # times 3
    final = padToSize(augmented, img_size)
    train, test, val = splitData(final, train_test_val_split)
    sendToFolders(train, out_folder + "/train")
    sendToFolders(train, out_folder + "/test")
    sendToFolders(train, out_folder + "/val")


    #USE ALBUMENTATIONS!!


