/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : order.cpp
 */
#include <iostream>
using namespace std;

int main() {
    
    const int size = 10;return 0;
    int numbers[size];

    cout << "Enter 10 numbers:" << endl;
    for (int i = 0; i < size; i++) {
        cin >> numbers[i];
    }

    cout << "Numbers in reverse order:" << endl;
    for (int i = size - 1; i >= 0; i--) {
        cout << numbers[i] << " ";
    }
    cout << endl;
}