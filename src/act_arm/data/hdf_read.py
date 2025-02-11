import h5py
import numpy as np
import matplotlib.pyplot as plt

# 打印出文件中所有的字段
def print_all_fields(hdf5_file):
    def print_name(name, obj):
        print(f"字段名: {name}")
        if isinstance(obj, h5py.Dataset):  # 如果是数据集
            print(f"数据集形状: {obj.shape}, 大小: {obj.size}")
    
    with h5py.File(hdf5_file, 'r') as file:
        file.visititems(print_name)

# 打印出指定字段的内容和大小
def print_field_content(hdf5_file, field_name):
    with h5py.File(hdf5_file, 'r') as file:
        if field_name in file:
            dataset = file[field_name]
            data = dataset[:]  # 获取数据内容
            print(f"字段名: {field_name}")
            print(f"数据集形状: {dataset.shape}, 大小: {dataset.size}")
            print("内容:")
            print(data)
        else:
            print(f"字段 {field_name} 不存在于文件中")

# 读取并绘制action和qpos数据
def plot_trajectory(hdf5_file):
    with h5py.File(hdf5_file, 'r') as file:
        actions = file['action'][:]
        qpos = file['observations/qpos'][:]
        
        time_steps = np.arange(len(actions))
        
        plt.figure(figsize=(12, 8))
        
        # 绘制action数据
        plt.subplot(211)
        for i in range(actions.shape[1]):
            plt.plot(time_steps, actions[:, i], label=f'action_{i}')
        plt.title('Actions over time')
        plt.xlabel('Time steps')
        plt.ylabel('Action values')
        plt.legend()
        plt.grid(True)
        
        # 绘制qpos数据
        plt.subplot(212)
        for i in range(qpos.shape[1]):
            plt.plot(time_steps, qpos[:, i], label=f'qpos_{i}')
        plt.title('Joint positions over time')
        plt.xlabel('Time steps')
        plt.ylabel('Joint positions')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()
        
        
# 示例用法
hdf5_file = './aloha_mobile_dummy/episode_4.hdf5'  # 替换为你的 HDF5 文件路径

# 打印所有字段
print_all_fields(hdf5_file)

# 打印指定字段内容
field_name = 'your_field_name'  # 替换为你要查看的字段名
# print_field_content(hdf5_file, field_name)

# 读取并绘制action和qpos数据
plot_trajectory(hdf5_file)


