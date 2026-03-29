"""
分析刚度试验数据：
1. 对 extension 数据做回归
2. 对 inter-finger 数据做回归
3. 用 inter-finger 的k值在 inter-finger 回归线上找 best 值
4. 用得到的 best 值在 extension 回归线上反向查找 k 值
"""

import pandas as pd
import numpy as np
from scipy import stats
import os

# 导入配置的k值
import sys
sys.path.insert(0, os.path.dirname(__file__))
from config import STIFFNESS_PRESETS

# ========================
# 1. 从 config 提取 k 值映射
# ========================
k_map = {
    'extension': {
        's': STIFFNESS_PRESETS['es']['k'],  # 180
        'm': STIFFNESS_PRESETS['em']['k'],  # 420
        'h': STIFFNESS_PRESETS['eh']['k'],  # 900
    },
    'inter-finger': {
        's': STIFFNESS_PRESETS['bs']['k'],  # 220
        'm': STIFFNESS_PRESETS['bm']['k'],  # 520
        'h': STIFFNESS_PRESETS['bh']['k'],  # 1100
    }
}

print("从 config 读取的 k 值:")
print(f"Extension: s={k_map['extension']['s']}, m={k_map['extension']['m']}, h={k_map['extension']['h']}")
print(f"Inter-finger: s={k_map['inter-finger']['s']}, m={k_map['inter-finger']['m']}, h={k_map['inter-finger']['h']}\n")

# ========================
# 2. 读取 CSV 文件
# ========================
csv_path = r'c:\Users\user\OneDrive\桌面\w_s1\WolverClaw_size_2022\Assets\Data\trial_stiffness\user_12_stiffness_trials.csv'
df = pd.read_csv(csv_path)

print(f"读取数据行数: {len(df)}")
print(f"列名: {df.columns.tolist()}\n")

# ========================
# 3. 数据预处理
# ========================
# 将 k 列的 s/m/h 转换为对应的数值
def convert_k_to_numeric(row):
    """根据Method和k值获取数值"""
    method = row['Method']
    k_letter = row['k']
    
    if 'extension' in method.lower():
        return k_map['extension'].get(k_letter)
    elif 'inter-finger' in method.lower():
        return k_map['inter-finger'].get(k_letter)
    return None

df['k_numeric'] = df.apply(convert_k_to_numeric, axis=1)

# 过滤有效数据（有best值）
df_valid = df[df['best'].notna()].copy()

print(f"有效数据（有best值）: {len(df_valid)} 行\n")

if len(df_valid) < 2:
    print("警告：有效数据不足，无法进行回归分析")
    print("请确保CSV文件中已填入best值")
else:
    # ========================
    # 4. 分别提取 extension 和 inter-finger 数据
    # ========================
    ext_data = df_valid[df_valid['Method'].str.contains('extension', case=False, na=False)].copy()
    inter_data = df_valid[df_valid['Method'].str.contains('inter-finger', case=False, na=False)].copy()
    
    print(f"Extension 数据: {len(ext_data)} 行")
    print(f"Inter-finger 数据: {len(inter_data)} 行\n")
    
    # ========================
    # 5. 生成回归线
    # ========================
    results = {}
    
    for name, data in [('extension', ext_data), ('inter-finger', inter_data)]:
        if len(data) >= 2:
            x = data['k_numeric'].values
            y = data['best'].values
            
            # 移除 NaN
            mask = ~(np.isnan(x) | np.isnan(y))
            x = x[mask]
            y = y[mask]
            
            if len(x) >= 2:
                # 线性回归
                slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)
                
                results[name] = {
                    'slope': slope,
                    'intercept': intercept,
                    'r_value': r_value,
                    'p_value': p_value,
                    'std_err': std_err,
                    'data': data
                }
                
                print(f"=== {name.upper()} 回归结果 ===")
                print(f"方程: best = {slope:.6f} * k + {intercept:.6f}")
                print(f"R值: {r_value:.6f}")
                print(f"R²值: {r_value**2:.6f}")
                print(f"p值: {p_value:.6e}")
                print(f"标准误差: {std_err:.6f}\n")
                
                # 绘制数据点信息
                print(f"数据范围:")
                print(f"  k 范围: {x.min():.1f} - {x.max():.1f}")
                print(f"  best 范围: {y.min():.6f} - {y.max():.6f}\n")
    
    # ========================
    # 6. 映射过程
    # ========================
    if 'inter-finger' in results and 'extension' in results:
        print("=" * 60)
        print("映射过程:")
        print("=" * 60)
        
        inter_slope = results['inter-finger']['slope']
        inter_intercept = results['inter-finger']['intercept']
        
        ext_slope = results['extension']['slope']
        ext_intercept = results['extension']['intercept']
        
        # inter-finger 的3个k值
        inter_k_values = [
            k_map['inter-finger']['s'],  # 220
            k_map['inter-finger']['m'],  # 520
            k_map['inter-finger']['h'],  # 1100
        ]
        inter_k_labels = ['s', 'm', 'h']
        
        print("\n步骤 1: 从 inter-finger 回归线取值")
        print("-" * 60)
        
        inter_best_values = []
        for k_val, label in zip(inter_k_values, inter_k_labels):
            best_val = inter_slope * k_val + inter_intercept
            inter_best_values.append(best_val)
            print(f"inter-finger {label} (k={k_val:.0f}): best = {best_val:.6f}")
        
        print("\n步骤 2: 用 inter-finger 得到的 best 值，反向查 extension k 值")
        print("-" * 60)
        
        # 反向回归：k = (best - intercept) / slope
        extension_k_values = []
        
        for i, (best_val, label) in enumerate(zip(inter_best_values, inter_k_labels)):
            if ext_slope != 0:
                k_val = (best_val - ext_intercept) / ext_slope
                extension_k_values.append(k_val)
                print(f"inter-finger {label} 的 best={best_val:.6f} -> extension k = {k_val:.2f}")
            else:
                print(f"警告: extension 回归线斜率为0，无法反向计算")
        
        # ========================
        # 7. 总结结果
        # ========================
        print("\n" + "=" * 60)
        print("最终映射结果:")
        print("=" * 60)
        
        print("\ninter-finger k 值 -> extension k 值:")
        for label, inter_k, ext_k in zip(inter_k_labels, inter_k_values, extension_k_values):
            print(f"  {label}: inter-finger k={inter_k:.0f} -> extension k={ext_k:.2f}")
        
        print(f"\n总共 {len(extension_k_values)} 个映射值")
        print(f"Extension 对应的马达 k 值总结: {extension_k_values}")
        
        # ========================
        # 8. 可视化数据（可选，生成文本输出）
        # ========================
        print("\n" + "=" * 60)
        print("数据可视化：")
        print("=" * 60)
        
        print("\nExtension 数据点:")
        ext_data_sorted = results['extension']['data'].sort_values('k_numeric')
        for idx, row in ext_data_sorted.iterrows():
            print(f"  k={row['k_numeric']:.0f}, best={row['best']:.6f}")
        
        print("\nInter-finger 数据点:")
        inter_data_sorted = results['inter-finger']['data'].sort_values('k_numeric')
        for idx, row in inter_data_sorted.iterrows():
            print(f"  k={row['k_numeric']:.0f}, best={row['best']:.6f}")
