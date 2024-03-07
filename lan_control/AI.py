import numpy as np
import pandas as pd
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import MinMaxScaler

# Load your data
data = pd.read_csv('/path/to/your/heatmap_data.csv')

# Preprocess the data
# Here, you might need to apply filtering to remove noise
# For simplicity, I'm replacing -8 with NaN and then forward filling
data.replace(-8, np.nan, inplace=True)
data.fillna(method='ffill', inplace=True)

# Normalize the data
scaler = MinMaxScaler(feature_range=(0, 1))
scaled_data = scaler.fit_transform(data)

# Prepare the dataset for LSTM
def create_dataset(X, y, time_steps=1):
    Xs, ys = [], []
    for i in range(len(X) - time_steps):
        v = X.iloc[i:(i + time_steps)].values
        Xs.append(v)
        ys.append(y.iloc[i + time_steps])
    return np.array(Xs), np.array(ys)

time_steps = 5
X, y = create_dataset(pd.DataFrame(scaled_data), pd.DataFrame(scaled_data), time_steps)

# Split the data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=0)

# Build the LSTM model
model = Sequential()
model.add(LSTM(50, activation='relu', input_shape=(X_train.shape[1], X_train.shape[2])))
model.add(Dense(110)) # 110 neurons, one for each sensor

# Compile the model
model.compile(optimizer='adam', loss='mean_squared_error')

# Train the model
model.fit(X_train, y_train, epochs=30, batch_size=32, validation_split=0.1, verbose=1)

# Evaluate the model
model.evaluate(X_test, y_test)
