#include <iostream>
using namespace std;
int main() {
    int var = 10;
    int *ptr = &var;
    cout << "var before = " << var << endl;
    *ptr = 20;
    cout << "var after = " << var << endl;
    return 0;
}