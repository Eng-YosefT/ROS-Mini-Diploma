#include <iostream>
#include <cstring>
using namespace std;

int main() {
    string str;
    cout << "Enter Your first name: ";
    cin >> first_name;
    cout << "Enter Your last name: ";
    cin >> last_name;   

    strleth(first_name);
    cout << "Length of first name is: " << first_name.size() << endl;   
    cout << "Length of last name is: " << last_name.size() << endl;
    cout << "Length of full name is: " << first_name.size() + last_name.size();
    return 0;       
   
}