import pandas as pd
import matplotlib.pyplot as plt

# Загружаем данные из CSV файла
df = pd.read_csv('Flight_data.csv',delimiter=";")  # Замените 'your_data.csv' на имя вашего CSV файла

# Создаем фигуру и сетку
f = plt.figure(constrained_layout=True)
gs = f.add_gridspec(4, 3)

# Определяем список столбцов для построения графиков
columns = ['pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z', 
           'vel_x', 'vel_y', 'vel_z', 'ang_vel_x', 'ang_vel_y', 'ang_vel_z']

ax1 = f.add_subplot(gs[0, :-1])
ax1.plot(df["vel_x"])
ax1.grid()
ax1.set_title('vel_x')

ax2 = f.add_subplot(gs[1, :-1])
ax2.plot(df["vel_y"],"g")
ax2.grid()
ax2.set_title('vel_y')

ax3 = f.add_subplot(gs[2, :-1])
ax3.plot(df["vel_z"], "r")
ax3.grid()
ax3.set_title('vel_z')

# Отображаем графики
plt.show()