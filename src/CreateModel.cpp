#include "GRT/GRT.h"
using namespace GRT;

int main (int argc, const char * argv[])
{

  //Create a new ANBC instance
  ANBC anbc;
  anbc.setNullRejectionCoeff( 10 );
  anbc.enableScaling( true );
  anbc.enableNullRejection( true );

  //Load some training data to train the classifier
  ClassificationData trainingData;

  if( !trainingData.load("data.txt") ){
      cout << "Failed to load training data!\n";
      return EXIT_FAILURE;
  }

  //Train the classifier
  if( !anbc.train( trainingData ) ){
      cout << "Failed to train classifier!\n";
      return EXIT_FAILURE;
  }

  //Save the ANBC model to a file
  if( !anbc.save("ASLModel_ABDI.grt") ){
      cout << "Failed to save the classifier model!\n";
      return EXIT_FAILURE;
  }
}
