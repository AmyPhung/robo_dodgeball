import os
import numpy as np  # Matrix handling
import tensorflow as tf
from matplotlib import pyplot
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense

# split a multivariate sequence into samples
def split_sequences(sequences, n_steps):
  """SPLITS DATA, exepects model outputs in the first column"""
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
dataset_names = ["009_2ball_random_1.5x5_vector_keyboard_nathan.npy",
                 "010_2ball_random_1.5x5_vector_keyboard_nathan.npy",
                 "011_2ball_random_1.5x5_vector_keyboard_nathan.npy",
                 "012_2ball_random_1.5x5_vector_keyboard_nathan.npy"]
n_steps = 8
hidden = 15
epochs = 100
mirror = True
use_origin = False
if __name__ == "__main__":
    input = None
    output = None
    print("_{:02d}".format(n_steps))
    save_name = "_{:02d}-{:03d}".format(n_steps, hidden)
    for dataset in dataset_names:
        save_name += "_" + dataset.split("_")[0]
        raw_data = np.load(dataset_loc + dataset)
        if not use_origin:
            # Trim the last robot position columns
            raw_data = raw_data[:, :9]
        file_input, file_output = split_sequences(raw_data, n_steps)
        n_features = file_input.shape[2]
        if input is not None:
            input = np.concatenate((input, file_input))
            output = np.concatenate((output, file_output))
        else:
            input = file_input
            output = file_output
        # Now add the mirrored
        if mirror:
            mirrored_data = np.copy(raw_data)
            cols_to_swap = (0, 1, 3, 5, 7) # Only reverse the motor command and things along that axis
            mirrored_data[:, cols_to_swap] *= -1
            try:
                # Wrap the robot position in a try except so that it won't error if it's in the dataset either way
                mirrored_data[:,9] *= -1
            except:
                pass
            mirrored_file_input, mirrored_file_output = split_sequences(mirrored_data, n_steps)
            input = np.concatenate((input, mirrored_file_input))
            output = np.concatenate((output, mirrored_file_output))

    print("Model Input Shape: ", input.shape)
    print("Model Output Shape: ", output.shape)
    print("Num Features: ", n_features)

    """
    MAKE THE MODEL!!
    """
    model = Sequential()
    model.add(LSTM(hidden, activation='relu', input_shape=(n_steps, n_features)))
    model.add(Dense(1))
    model.compile(optimizer='adam', loss='mse', metrics=['accuracy'])

    """
    Train the model!
    """
    # fit model
    #model.fit(input, output, epochs=1000, verbose=1)
    history = model.fit(input, output, epochs=epochs, validation_split=0.2)
    # plot train and validation loss
    pyplot.plot(history.history['loss'][2:])
    pyplot.plot(history.history['val_loss'][2:])
    pyplot.title('model train vs validation loss using nsteps:{} hidden: {}'.format(n_steps, hidden))
    pyplot.ylabel('loss')
    pyplot.xlabel('epoch')
    pyplot.legend(['train', 'validation'], loc='upper right')
    pyplot.show()

    export_path_sm = model_loc + "LSTM" + save_name
    print("Saving to: ", export_path_sm)
    tf.saved_model.save(model, export_path_sm)
