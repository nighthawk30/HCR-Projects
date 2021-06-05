Nathan Taylor
4/28/21
libsvm training and predicting for skeleton based representations

To Run:
1. install libsvm and gnuplot
2. run python3 skeleton_svm.py

Hyperparameters:
Custom: c = 32, g = .001953125
RAD: c = .03125, g = .03125

Accuracy:
Custom: 52.1%
RAD: 60.4%

Notes:

The files and file paths are hardcoded into skeleton_svm.py. Move the representation data
into the desired location and then alter the file path in skeleton_svm.py to match. You will need
to update the file in skeleton_svm.py depending on the title of the representation that you made.

The prediction output files are formatted so that on each line is a number corresponding to one of the actions. The prediction is for the label  on the same line in the test file. In order to get this succinctly, I wrote another file wordone.cpp that prints out the first character of each line in the test data file. It turns out that this is not necessary since the actions simply go from 0 to 5 and then repeat for 48 total instances.
