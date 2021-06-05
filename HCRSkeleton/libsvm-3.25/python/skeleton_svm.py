from libsvm.svmutil import *

def main():
    ty, tx = svm_read_problem('../../D2_nathan_taylor/rad_d2')
    m = svm_train(ty, tx, '-c .03125 -g .03125')
    py, px = svm_read_problem('../../D2_nathan_taylor/rad_d2.t')
    p_labels, p_acc, p_vals = svm_predict(py, px, m)
    #by the forgotten gods, it works
    print(p_labels)
    s_labels = []
    for x in p_labels:
        s_labels.append(str(x) + "\n")
    file = open("rad_d2.t.predict", 'w')
    file.writelines(s_labels)
    file.close()
main()
#for the love of god






#//-c .03125 -g .03125

#//use same read problem function
#//copy parse_command_line


#/*
#easy.py for template
#scipy
#in python folder for python readme and svmutil for high level functions
#empty model
#train it on something
#model = svmtrain


#y, x = svm_read_problem("data.txt")

#prob = svm_problem(y, x)
#param = svm_parameter('')
#m = svm_train(y, x, '')
#p_labels, p_acc, p_vals = svm_predict(y, x, m)

  
#  //FUNCTION PROCESS

#Train:
#-create svm_parameter struct
#-create svm_problem struct
#-create svm_model struct pointer
#-create svm_node struct pointer

#1. Set parameter values
#2. Get file name
#3. Read file into list of labels, indexes and data
#4. Run svm_train on problem and parameter
#5. save model to a file

#Predict:
#-create svm_node struct pointer
#-create svm_model struct pointer

#1. Set input and output files
#2. Read in each line from the input file
#3. Separate the labels from the data: svm_get_labels(model,labels)
#4. run svm_predict on the model and node
#5. Compare the predicted label to the target label
#6. Generate statistics based on correctness
#*/
  
