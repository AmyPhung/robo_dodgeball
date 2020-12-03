import os
import numpy as np  # Matrix handling
import tensorflow as tf
from matplotlib import pyplot
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense

# split a multivariate sequence into samples
def split_sequences(sequences, n_steps):
  """SPLITS DATA, exepects omdel outputs in the first column"""
  X, y = list(), list()
  for i in range(len(sequences)):
    # find the end of this pattern
    end_ix = i + n_steps
    # check if we are beyond the dataset
    if end_ix > len(sequences):
      break
    # gather input and output parts of the pattern
    seq_x, seq_y = sequences[i:end_ix, 1:], sequences[end_ix-1, 0]
    X.append(seq_x)
    y.append(seq_y)
  return np.array(X), np.array(y)

dataset_loc = os.path.expanduser("~/catkin_ws/src/ml_comprobofinal/datasets/")
model_loc = os.path.expanduser("~/catkin_ws/src/ml_comprobofinal/ml_models/")
dataset_name = "LSTM_05_20_002.npy"
n_steps = 5

if __name__ == "__main__":
    raw_data = np.load(dataset_loc + dataset_name)
    input, output = split_sequences(raw_data, n_steps)
    n_features = input.shape[2]

    print("Model Input Shape: ", input.shape)
    print("Model Output Shape: ", output.shape)
    print("Num Features: ", n_features)

    """
    MAKE THE MODEL!!
    """
    model = Sequential()
    model.add(LSTM(20, activation='relu', input_shape=(n_steps, n_features)))
    model.add(Dense(1))
    model.compile(optimizer='adam', loss='mse', metrics=['accuracy'])

    """
    Train the model!
    """
    # fit model
    #model.fit(input, output, epochs=1000, verbose=1)
    history = model.fit(input, output, epochs=50, validation_split=0.2)
    # plot train and validation loss
    pyplot.plot(history.history['loss'][1:])
    pyplot.plot(history.history['val_loss'][1:])
    pyplot.title('model train vs validation loss')
    pyplot.ylabel('loss')
    pyplot.xlabel('epoch')
    pyplot.legend(['train', 'validation'], loc='upper right')
    pyplot.show()

    export_path_sm = model_loc + dataset_name.split('.')[0]
    print("Saving to: ", export_path_sm)
    tf.saved_model.save(model, export_path_sm)


