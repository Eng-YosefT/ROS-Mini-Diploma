/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : even.cpp
 */
#include <iostream>
using namespace std;

int main() {
    for (int i = 0; i <= 20; i++) {
        if (i % 2 == 0) {
            cout << i << " ";
        }
    }
    cout << endl;
    return 0;
}