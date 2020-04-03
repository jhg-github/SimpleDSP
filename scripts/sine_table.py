import numpy
import math
import matplotlib.pyplot as plt

L = 2048
x_n = numpy.arange(0,2*math.pi,2*math.pi/L)
sine_table=[]
c=0
for x in x_n:
    sine_n = int(math.sin(10*x)*512 + 2047)
    print(sine_n, end=', ')
    sine_table.append(sine_n)
    c +=1
    if c == 10:
        c=0
        print()
plt.plot(sine_table, 'x')
plt.show()
print()
print(len(sine_table))

"""
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 256 + n++);
    HAL_Delay(1);
"""