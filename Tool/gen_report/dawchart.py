import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("./gps.txt",sep= " ")

print(df['x'][0:3])

# plt.plot(df['x'][0:10])
# plt.ylabel("axis X")
# plt.xlabel("time")

        # plt.plot([1,2,3,4], [1,4,9,16], 'ro')
# plt.axis([0, 6, 0, 20])
# plt.show()

# df.info()
# print(df.columns)
# print(df['x'][0:2])
# for index, row in df.iterrows():
#     print(index, row)
