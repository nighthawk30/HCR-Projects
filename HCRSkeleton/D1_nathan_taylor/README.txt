Nathan Taylor
4/16/21
Skeleton Representation

To Compile/Run:
1. Install g++ compiler
2. Run: g++ skeleton.cpp
3. Execute: ./a.out

This will access the data in the dataset folder and create 2 representations for the training and test set of data.

The joints used for the relative angles and distances representation were 1,8,16,20,12 and 4.
The histograms are computed by creating bins equal to the square root of the total number of data points rounded up. The width of the bins is the range of the data divided by the number of bins.
First the distances and angles are computed and put into a vector. Then, for each frame and each distance in an instance, I check which bin the data point falls under and I add 1/# of instances to the bin count. This normalizes the histogram. Then I concatenate all the histograms for an instance and write it to an output file.

The second representation I used was simply the distance from each of the joints to the hip joint (1). Full disclosure, I'm using it because it is easy.
