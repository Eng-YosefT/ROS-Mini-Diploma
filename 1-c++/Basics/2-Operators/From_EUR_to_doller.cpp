/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : From_EUR_to_doller.cpp
 */
#include <iostream>
using namespace std;

int main() {
    cout << "welcome " << endl;
    cout << "Enter amount in EUR: ";
    double eur;
    cin >> eur;
    double usd = eur * 1.19; 
    cout << "Equivalent in USD: " << usd << endl;
    return 0;
}