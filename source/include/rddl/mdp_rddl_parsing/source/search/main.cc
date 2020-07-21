#include "extractor.h"

#include <cstdlib>
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char** argv) {
    if (argc < 2) {
	cout << "incorrect usage" << endl; //TODO: print out usage information
	return 1;
    }
    /******************************************************************
                          Parse command line
    ******************************************************************/

    string problemDir = string(argv[1]);
    string problemName = string(argv[2]);

    rddlmdp::Extractor e(problemName, problemDir);
    e.exploring();

    // Create connector to rddlsim and run
    //    IPPCClient* client = new IPPCClient(hostName, port, pruning_prob);
    //client->run(problemFileName, plannerDesc);

    return 0;
}
