#include "GRT/GRT.h"
using namespace GRT;

int main (int argc, const char * argv[])
{

  //Create a new RandomForests instance
  RandomForests forest;

  //Set the number of trees in the forest
  forest.setForestSize( 10 );

  //Set the number of random candidate splits that will be used to choose the best splitting values
  //More steps will give you a better model, but will take longer to train
  forest.setNumRandomSplits( 100 );

  //Set the maximum depth of the tree
  forest.setMaxDepth( 10 );

  //Set the minimum number of samples allowed per node
  forest.setMinNumSamplesPerNode( 10 );

  //Load some training data to train the classifier
  ClassificationData trainingData;

  if( !trainingData.load(argv[1]) ){
      cout << "Failed to load training data!\n";
      return EXIT_FAILURE;
  }

  //Train the classifier
  if( !forest.train( trainingData ) ){
      cout << "Failed to train classifier!\n";
      return EXIT_FAILURE;
  }

  //Save the ANBC model to a file
  if( !forest.save("ASLModel_ABDI.grt") ){
      cout << "Failed to save the classifier model!\n";
      return EXIT_FAILURE;
  }
}
