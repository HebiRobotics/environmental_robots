import matplotlib.pyplot as plt
import json

result = None
with open('material_result.json', 'r') as f:
    result = json.loads(f.read())

settings = result['scanSettings']
spectrum = result['runSpectrumResult']
print(settings)

plt.plot(spectrum[0], spectrum[1])
plt.show()
