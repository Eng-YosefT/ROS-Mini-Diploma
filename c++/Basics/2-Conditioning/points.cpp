/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : points.cpp
 */
#include <iostream>
using namespace std;

int main() {
    
    int grade;
    cout << "Enter your score: ";
    cin >> grade;

    if (grade < 0 || grade > 100) {
        cout << "Invalid score. Please enter a value between 0 and 100." << endl;
    } else if (grade < 50) {
        cout << "Rating: Failed" << endl;
    } else if (grade < 65) {
        cout << "Rating: Pass" << endl;
    } else if (grade < 75) {
        cout << "Rating: Good" << endl;
    } else if (grade < 85) {
        cout << "Rating: Very Good" << endl;
    } else {
        cout << "Rating: Excellent" << endl;
    }
    return 0;
}