/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : result.cpp
 */
#include <iostream>
using namespace std;

int main() {
    int answer;
    do {
        cout << "What is the result of 3 * 4? ";
        cin >> answer;
        if (answer == 12) {
            cout << "Thanks" << endl;
        } else {
            cout << "Try again" << endl;
        }
    } while (answer != 12);
    return 0;
}