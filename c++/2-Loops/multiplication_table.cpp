/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : multiplication_table.cpp
 * Description: 
 */
#include <iostream>
using namespace std;

int main() {
    for (int i = 1; i <= 10; ++i) {
        cout << "Multiplication Table for " << i << ":" << endl;
        for (int j = 1; j <= 10; ++j) {
            cout << i << " x " << j << " = " << i * j << endl;
        }
        cout << "--------------------------" << endl;
    }
        for (int j = 1; j <= 10; ++j) {
            cout << i << " x " << j << " = " << i * j << "\t";
        }
        cout << endl;
    }
    return 0;
}