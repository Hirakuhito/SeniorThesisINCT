import time

import numpy as np
import pybullet as p
import pybullet_data as pd
import trackMaker.point_info_generator as pg


def main():
    print("# Welcome")

    a = pg.gen_center_point(100, 50)
    print(f"生成されたポイントの総数: {len(a)}")
    
    

if __name__ == "__main__":
    main()