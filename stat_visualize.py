import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
df = pd.read_csv('goalstat.csv')

# Create a boxplot
plt.figure(figsize=(10, 6))
df.boxplot(column=[0])

# Show the plot
plt.show()