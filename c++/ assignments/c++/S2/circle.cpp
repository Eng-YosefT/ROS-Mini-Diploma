/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : circle.cpp
 * Description: 
 */
#include <iostream>
using namespace std;

int main() {
    double radius, area, perimeter;
    const double PI = 3.14159;

    cout << "Enter the radius of the circle: ";
    cin >> radius;

    area = PI * radius * radius;
    perimeter = 2 * PI * radius;

    cout << "Area of the circle: " << area << endl;
    cout << "Perimeter of the circle: " << perimeter << endl;
    return 0;
}