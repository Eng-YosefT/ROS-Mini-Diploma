/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : area.cpp
 */
#include <iostream>
using namespace std;

int main() {
    char choice;
    do {
        double height, width, area;

        cout << "Enter height: ";
        cin >> height;

        cout << "Enter width: ";
        cin >> width;

        area = height * width;
        cout << "The area is: " << area << endl;

        cout << "Do you want to perform another calculation? (y/n): ";
        cin >> choice;

    } while (choice == 'y' || choice == 'Y');
    return 0;
}