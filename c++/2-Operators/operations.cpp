/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : operations.cpp
 */
#include <iostream>
using namespace std;

int main() {
    int num1, num2;

    cout << "Enter the first integer: ";
    cin >> num1;

    cout << "Enter the second integer: ";
    cin >> num2;

    cout << num1 <<" > "<< num2 <<" : "<<  (num1 > num2) << endl;
    cout << num1 <<" < "<< num2 <<" : "<<  (num1 < num2) << endl;
    cout << num1 <<" >= "<< num2 <<" : "<< (num1 >= num2) << endl;
    cout << num1 <<" <= "<< num2 <<" : "<< (num1 <= num2) << endl;
    cout << num1 <<" == "<< num2 <<" : "<< (num1 == num2) << endl;
    cout << num1 <<" != "<< num2 <<" : "<< (num1 != num2) << endl;
    cout << "Left shift " << (num1 << num2) << endl;
    cout << "Right shift " << (num1 >> num2) << endl;
    cout << "Bitwise AND "<< num1 <<" & "<< num2 <<" : " << (num1 & num2) << endl;
    cout << "Bitwise OR "<< num1 <<" | "<< num2 <<" : " << (num1 | num2) << endl;
    cout << "Bitwise XOR "<< num1 <<" ^ "<< num2 <<" : " << (num1 ^ num2) << endl;
    cout << "Bitwise NOT "<< num1 <<" ~ "<< num2 <<" : " << (~num1) << endl;
    cout << "Bitwise NOT "<< num1 <<" ~ "<< num2 <<" : " << (~num2) << endl;
    cout << "Logical AND "<< num1 <<" && "<< num2 <<" : " << (num1 && num2) << endl;
    cout << "Logical OR "<< num1 <<" || "<< num2 <<" : " << (num1 || num2) << endl;
    cout << "Logical NOT "<< num1 <<" ! "<< num2 <<" : " << (!num1) << endl;

    return 0;
}