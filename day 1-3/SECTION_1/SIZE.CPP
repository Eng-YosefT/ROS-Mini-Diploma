#include <iostream>
using namespace std;

int main() {
    cout << "The size of an int is: " << sizeof(int) << " bytes.\n";
    cout << "The size of a short int is: " << sizeof(short int) << " bytes.\n";
    cout << "The size of a long int is: " << sizeof(long int) << " bytes.\n";
    cout << "The size of a char is: " << sizeof(char) << " bytes.\n";
    cout << "The size of a float is: " << sizeof(float) << " bytes.\n";
    cout << "The size of a double is: " << sizeof(double) << " bytes.\n";
    cout << "The size of a wchar_t is: " << sizeof(wchar_t) << " bytes.\n";
    return 0;
}