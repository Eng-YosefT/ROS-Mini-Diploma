/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : less_100.cpp
 * Description: 
 */
#include <iostream>
using namespace std;

int main() {
    int number;
    do {
        cout << "Enter a number less than 100: ";
        cin >> number;
        if (number >= 100) {
            cout << "The number must be less than 100. Try again." << endl;
        }
    } while (number >= 100);
    return 0;
}