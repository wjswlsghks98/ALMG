## Import
#
import os
import argparse
import numpy as np
import tensorflow as tf
from sklearn.metrics import confusion_matrix, accuracy_score
from sklearn.model_selection import StratifiedKFold
from keras.utils import to_categorical


## Configuration
#
parser = argparse.ArgumentParser()
parser.add_argument('-t', '--testTrip', type=str, default = '2030')
args = parser.parse_args()


## Load data for Detection model
#
testtrip = args.testTrip
data_path = 'D:/SJ_Dataset/MatlabFiles/github/EventDetection/TrajectoryBasedDetection/src/Detection/data/'
type = 'EF' #IS or EF
file_list = [file for file in os.listdir(data_path) if file.startswith(type) & ~(testtrip in file)]

data = np.array([])
for filename in file_list:
    file_path = os.path.join(data_path, filename)
    if data.size == 0:
        data = np.loadtxt(file_path, delimiter = ',')
    else:
        data = np.vstack((data,np.loadtxt(file_path, delimiter=',')))
if type == 'IS':
    features = data[:,0:-1].reshape([data.shape[0],-1,100]) ## val
    model_CNN = tf.keras.models.Sequential([
        tf.keras.layers.Conv1D(filters=64,
                               kernel_size=9,
                               activation='relu',
                               padding='SAME',
                               input_shape=features.shape[1:]),

        tf.keras.layers.MaxPool1D(pool_size=2, strides=2),

        tf.keras.layers.LSTM(128),

        tf.keras.layers.Dense(128, activation='relu'),

        tf.keras.layers.Dense(2, activation='softmax')
    ])
elif type == 'EF':
    features = data[:, 0:-1].reshape([data.shape[0], -1, 11])  ## val
    shape = features.shape[1:] + (1,)
    model_CNN = tf.keras.models.Sequential([
        tf.keras.layers.Conv2D(filters=16,
                               kernel_size=(3, 3),
                               activation='relu',
                               padding='SAME',
                               input_shape=shape),

        tf.keras.layers.Conv2D(filters=32,
                               kernel_size=(3, 3),
                               activation='relu',
                               padding='SAME'),

        tf.keras.layers.Flatten(),

        tf.keras.layers.Dense(128, activation='relu'),

        tf.keras.layers.Dense(2, activation='softmax')
    ])
np.savetxt('output.csv', data, delimiter=',', fmt='%f')
labels = data[:,-1].reshape(-1,1)
nonzero_idx = np.where(labels !=0)[0]
bi_labels = data[:,-1].reshape(-1,1)
bi_labels[nonzero_idx] = 1
np.savetxt('output_bi.csv', data, delimiter=',', fmt='%f')

## CNN Detection model
#
model_CNN.compile(optimizer= tf.keras.optimizers.Adam(learning_rate=0.00005),
                  loss = 'categorical_crossentropy',
                  metrics= ['accuracy'])
model_CNN.summary()


## Cross Validation
#
skfold = StratifiedKFold(n_splits=5,random_state=42,shuffle=True)
n = 0
acc_sum = 0
cm_sum = 0

for train_idx, test_idx in skfold.split(features,bi_labels):
    x_train, x_test = features[train_idx], features[test_idx]
    y_train, y_test = bi_labels[train_idx], bi_labels[test_idx]
    y_train = to_categorical(y_train)

    model_CNN.fit(x_train,y_train,epochs = 150, batch_size=5,verbose=0)

    predict_cvCNN = model_CNN.predict(x_test)
    predict_cvCNN = np.argmax(predict_cvCNN, axis=1)
    acc_cvCNN = accuracy_score(y_test, predict_cvCNN)
    acc_sum += acc_cvCNN
    cm_cvCNN = confusion_matrix(y_test,predict_cvCNN)
    cm_sum += cm_cvCNN

    n +=1

print("Average Accuracy (Detection model)= ",acc_sum/n)
print("Average Confusion Matrix (Detection model)=")
print(cm_sum)


## Load data for Classification model
#
data_path_ = 'D:/SJ_Dataset/MatlabFiles/github/EventDetection/TrajectoryBasedDetection/src/Classification/data/'
type_ = 'IS'
file_list_ = [file for file in os.listdir(data_path_) if file.startswith(type_) & ~(testtrip in file)]
data_ = np.array([])
for filename in file_list_:
    file_path = os.path.join(data_path_, filename)
    if data_.size == 0:
        data_ = np.loadtxt(file_path, delimiter = ',')
    else:
        data_ = np.vstack((data_,np.loadtxt(file_path, delimiter=',')))

labels_ = data_[:,-1].reshape(-1,1)
nonzero_idx_ = np.where(labels_ !=0)[0]
if type_ == 'IS':
    features_ = data_[nonzero_idx_, 0:-1].reshape([nonzero_idx_.shape[0],-1,100])  ##val
elif type_ == 'EF':
    features_ = data_[nonzero_idx_, 0:-1].reshape([nonzero_idx_.shape[0], -1, 11])  ##val
labels_ = data_[nonzero_idx_,-1].reshape(-1,1)-1


## CNN Classification model
#
model2_CNN = tf.keras.models.Sequential([
    tf.keras.layers.Conv1D(filters = 64,
                           kernel_size = 9,
                           activation = 'relu',
                           padding = 'SAME',
                           input_shape = features_.shape[1:]),                       

    tf.keras.layers.MaxPool1D(pool_size = 2, strides= 2),

    tf.keras.layers.LSTM(128),

    tf.keras.layers.Dense(128, activation= 'relu'),

    tf.keras.layers.Dense(5, activation= 'softmax')
])
model2_CNN.compile(optimizer= tf.keras.optimizers.Adam(learning_rate=0.00005),
                  loss = 'categorical_crossentropy',
                  metrics= ['accuracy'])
model2_CNN.summary()


## Cross Validation
#
skfold = StratifiedKFold(n_splits=5,random_state=42,shuffle=True)
n = 0
acc_sum = 0
cm_sum = 0

for train_idx, test_idx in skfold.split(features_,labels_):
    x_train, x_test = features_[train_idx], features_[test_idx]
    y_train, y_test = labels_[train_idx], labels_[test_idx]
    y_train = to_categorical(y_train)

    model2_CNN.fit(x_train,y_train,epochs = 150, batch_size=5,verbose=0)

    predict_cvCNN = model2_CNN.predict(x_test)
    predict_cvCNN = np.argmax(predict_cvCNN, axis=1)
    acc_cvCNN = accuracy_score(y_test, predict_cvCNN)
    acc_sum += acc_cvCNN
    cm_cvCNN = confusion_matrix(y_test,predict_cvCNN)
    cm_sum += cm_cvCNN

    n +=1

print("Average Accuracy (Classification model)= ",acc_sum/n)
print("Average Confusion Matrix (Classification model)=")
print(cm_sum)

model_CNN.save('D:/SJ_Dataset/MatlabFiles/github/EventDetection/TrajectoryBasedDetection/src/Detection/model_CNN')
model2_CNN.save('D:/SJ_Dataset/MatlabFiles/github/EventDetection/TrajectoryBasedDetection/src/Classification/model_CNN')