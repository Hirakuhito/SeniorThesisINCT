import pandas as pd
import numpy as np
import time
import os

df = pd.DataFrame(np.random.randn(5, 3), columns=['A', 'B', 'C'])

try:
    for i in range(10):
        # 画面をクリア（Windowsなら 'cls', Mac/Linuxなら 'clear'）
        os.system('cls' if os.name == 'nt' else 'clear')
        
        # データの更新
        df += np.random.randn(5, 3) * 0.1
        
        print(f"Update: {i+1}")
        print(df)
        
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Stopped.")