import matplotlib.pyplot as plt
import random
import numpy as np
y_values = [round(p/28,ndigits=2) for p in range(29)]
print(y_values)
def parse_file(file_path):
    try:
        
        array = [[]]
        dict_arr = [{}]
        with open("./"+file_path, "r") as file:
            index = 0
            for line in file:
                # Remove leading/trailing whitespaces and split the line by commas
                numbers = line.strip().split(',')
                # Check if the line contains at least three comma-separated values
                if len(numbers) >= 3:
                    # Assuming the numbers are integers, you can convert them to integers like this:
                    array[-1].append((float(numbers[0]),float(numbers[2])))
                    dict_arr[-1][float(numbers[2])] = float(numbers[0])
                                        
                else:
                    array.append([])
                    dict_arr.append({})
        
        # Plot the data
        total_dict = {}
        for p in y_values:
            total_dict[p] = []
        
        for plot in dict_arr:
            for i in range(len(y_values))[::-1]:
                if y_values[i] in plot.keys() and y_values[i-1] not in plot.keys():
                    plot[y_values[i-1]] = plot[y_values[i]]

            for i in range(len(y_values)):
                if y_values[i] in plot.keys():
                    total_dict[y_values[i]].append(plot[y_values[i]])
        
        average_arr = [(sum(p)/len(p), v) for v, p in total_dict.items() if len(p) != 0]
        
        return average_arr
        

    except FileNotFoundError:
        print("File not found:", file_path)
    except Exception as e:
        print("An error occurred:", e)

# Example usage:
file_path = 'out.txt'  # Replace 'data.txt' with the path to your text file
average_arr = parse_file(file_path)

fig,ax = plt.subplots()

plt.xlabel("time (seconds)")
plt.ylabel("coverage (percent)")
ax.plot([p[0] for p in average_arr], [p[1] for p in average_arr])

plt.show()