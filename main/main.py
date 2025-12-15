import time

import numpy as np
import pybullet as p
import pybullet_data as pd
import trackMaker.track_info_generator as pg


def main():
    print("# Welcome")

    a = pg.gen_center_point(100, 50)
    print(f"生成されたポイントの総数: {len(a)}")
    print(f"{a}\n")

    b = pg.gen_mesh_data(a, 10, 50)
    print("--- 結果 ---")
    print(f"入力ポイント数: {len(a)}")
    print(f"出力メッシュデータ形状 (N, 4): {b.shape}")
    print("\n最初の3点の結果 (l_x, l_y, r_x, r_y):")
    print(b[:3])

    
if __name__ == "__main__":
    main()