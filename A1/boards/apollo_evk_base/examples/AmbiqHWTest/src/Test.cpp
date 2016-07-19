//============================================================================
// Name        : Test.cpp
// Author      : asha
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
using namespace std;

extern void Testmain(void);

int main() {
	int status = 0;
	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
	Testmain();
	return 0;
}
