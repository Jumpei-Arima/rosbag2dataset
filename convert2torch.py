import os
import argparse
import collections
import glob

from tqdm import tqdm
from PIL import Image

import numpy as np
import torch
import torchvision

def convert(time, img, vel):
    image = torchvision.transforms.ToTensor()(Image.open(img))
    velocity = torch.tensor(np.load(vel))
    scene = Scene(image, velocity)
    return scene
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir_name', type=str, default='dataset')
    args= parser.parse_args()
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), args.dir_name)
    path_list = os.listdir(os.path.join(path,"vel"))
    print(path)
    os.makedirs(os.path.join(path,"scenes"))
    time_list = list(set(path[0:20] for path in path_list))
    
    Scene = collections.namedtuple('Scene', ['image', 'velocity'])

    pbar = tqdm(total=len(time_list))
    for time in time_list:
        img_file_path = os.path.join(path,"img",(str(time)+".jpg"))
        vel_file_path = os.path.join(path,"vel",(str(time)+".npy"))
        scene = convert(time, img_file_path, vel_file_path)
        torch.save(scene, os.path.join(path, "scenes", (str(time)+".pt")))
        pbar.update(1)
    pbar.close()
