
def horizontalFlipAugment(samples):
    out = samples
    for sample in samples:
        image, label = sample
        flipped_img, flipped_label = horizontalFlip(image, label)
        out.append((flipped_img, flipped_label))
    return out

def rotateAugment(samples):
    out = samples
    for sample in samples:
        image, label = sample
        rotated_img, rotated_label = rotate(image, label, 90)
        out.append((rotated_img, rotated_label))
        rotated_img, rotated_label = rotate(image, label, 180)
        out.append((rotated_img, rotated_label))
        rotated_img, rotated_label = rotate(image, label, 270)
        out.append((rotated_img, rotated_label))
    return out

def cropAugment(samples):
    out = samples
    for sample in samples:
        image, label = sample
        cropped_img, cropped_label = crop(image, label, 'top-half')
        out.append((cropped_img, cropped_label))
    return out

def brightnessAugment(samples):
    out = samples
    for sample in samples:
        image, label = sample
        bright_img = change_brightness(image, 1)
        dark_img = change_brightness(image, -1)
        out.append((bright_img, label))
        out.append((dark_img, label))
    return out

def blurAugment(samples):
    out = samples
    for sample in samples:
        image, label = sample
        blurred_img = blur(image)
        out.append((blurred_img, label))
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
        red_img = change_color_balance(image, 'r')
        green_img = change_color_balance(image, 'g')
        out.append((blue_img, label))
        out.append((red_img, label))
        out.append((green_img, label))
        out.append((grayscale_img, label))
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
    augmented = rotateAugment(augmented) # times 4
    augmented = cropAugment(augmented) # times 2
    augmented = horizontalFlipAugment(data) # times 2
    augmented = brightnessAugment(augmented) # times 3
    augmented = blurAugment(augmented) # times 2
    augmented = noiseAugment(augmented) # times 2
    augmented = colorAugment(augmented) # times 4
    final = padToSize(augmented, img_size)
    train, test, val = splitData(final, train_test_val_split)
    sendToFolders(train, out_folder + "/train")
    sendToFolders(train, out_folder + "/test")
    sendToFolders(train, out_folder + "/val")


    #USE ALBUMENTATIONS!!


