import yaml
import numpy as np

with open('esp32_intrinsic.yml', 'r') as f:
    data = yaml.load(f, Loader=yaml.UnsafeLoader)

print(data['mtx'])