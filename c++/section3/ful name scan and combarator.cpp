#include <iostream>
#include <cstring>
using namespace std;
int main() {
    char full_name [] = "youssef Taha";
    char in_name[20];
    
    cout << "Enter Your full name: ";
    getline(cin, in_name);
    cout << "Your full name is: " << in_name << endl;
    cout << "---------------------------- " << endl;
    if (full_name == in_name) {
        cout << full_name << " and " << in_name << "is same " << endl;
        cout << "---------------------------- " << endl;

    } else {
        cout << "You are not welcome " << in_name << endl;
    }
    for 
}