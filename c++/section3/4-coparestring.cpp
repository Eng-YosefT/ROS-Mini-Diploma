#include <iostream>
#include <string>
using namespace std;
int main() {
    string full_name  = "youssef Taha";
    string first_name;
    string last_name;
    cout << "Enter Your first name: ";
    cin >> first_name;
    cout << "Enter Your last name: ";
    cin >> last_name;
    cout << "Your full name is: " << first_name + " " + last_name << endl;
    cout << "---------------------------- " << endl;
}