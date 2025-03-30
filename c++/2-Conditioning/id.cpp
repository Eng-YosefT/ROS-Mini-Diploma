/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : id.cpp
 */
#include <iostream>
using namespace std;

int main() {
    int id;
    cout << "Enter your ID: ";
    cin >> id;

    switch (id) {
        case 1234:
            cout << "Ahmed" << endl;
            break;
        case 5677:
            cout << "Youssef" << endl;
            break;
        case 1145:
            cout << "Mina" << endl;
            break;
        default:
            cout << "Wrong ID" << endl;
            break;
    }
    return 0;
}