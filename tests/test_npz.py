import numpy as np

# 加载npz文件
data = np.load('/BipedChallenge/docs/dev-branch-copy/leju_kuavo_tongverse-lite/examples/task_3_solver/task3_cmd.npz')

# 打印所有包含的文件名
print(data.files)

# 遍历所有文件并打印内容
for file in data.files:
    print(f"{file}:")
    print(data[file].shape)
    print(data[file])