import os
import collections
import glob

from tqdm import tqdm
from PIL import Image

import numpy as np
import torch
import torchvision

from multiprocessing import Pool

def convert(time_idx):
    img_files = sorted(glob.glob(f"/home/amsl/workspace/rosbag2dataset/dataset/img/{time_idx}.jpg"))
    vel_files = sorted(glob.glob(f"/home/amsl/workspace/rosbag2dataset/dataset/vel/{time_idx}.npy"))
    images = torch.stack([torchvision.transforms.ToTensor()(Image.open(img)) for img in img_files])
    velocities = []
    for vel_path in vel_files:
        velocities.append(np.load(vel_path))
    velocities = torch.tensor(velocities)
    scene = Scene(images, velocities)
    torch.save(scene, f"/home/amsl/workspace/rosbag2dataset/dataset/scenes/{int(time_idx)}.pt")
    
if __name__ == "__main__":
    path = os.path.dirname(os.path.abspath(__file__)) + "/dataset/vel"
    path_list = os.listdir(path)
    time_list = list(set(path[0:19] for path in path_list))
    
    Scene = collections.namedtuple('Scene', ['image', 'velocity'])
    
    p = Pool()
    p.map(convert, time_list)
