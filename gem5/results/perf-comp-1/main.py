import pandas as pd

file_path = "../../m5out/ticks_output.txt"

df = pd.read_csv(file_path, sep=":", header=None, names=["key", "value"])

print(df)
