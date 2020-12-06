"""
Visualizes metrics of a trained nueral network
"""

# Pick which model to visualize metrics for
model_name = "standard_999"

import pickle                      # Loading in data
from helpers import get_ws_path    # Make life easier
import matplotlib.pyplot as plt # For plotting

""" Load in the metrics data """

ws_path = get_ws_path()
models_folder = ws_path + "ml_models/"

metrics_file = open(models_folder + model_name + "/metrics.pkl", 'rb')
metrics_dict = pickle.load(metrics_file)
for key in metrics_dict:
    print(key)
metrics_file.close()

"""
Visualize how the loss changes over the different epochs
"""

if "epochs" in metrics_dict and "loss_list" in metrics_dict:
    plt.plot(metrics_dict["epochs"], metrics_dict["loss_list"])
    plt.xlabel("Epoch")
    plt.ylabel("Loss")
    plt.title("Loss Over Epochs")
    plt.show()
