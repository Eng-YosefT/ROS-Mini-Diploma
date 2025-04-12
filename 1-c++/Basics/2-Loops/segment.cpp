/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : segment.cpp
 */
#include <iostream>
using namespace std;

int main() {
    int sum = 0, number;
    cout << "Enter 10 numbers:" << endl;
    for (int i = 0; i < 10; i++) {
        cin >> number;
        sum += number;
    }
    cout << "The summation of the numbers is: " << sum << endl;
    return 0;
}