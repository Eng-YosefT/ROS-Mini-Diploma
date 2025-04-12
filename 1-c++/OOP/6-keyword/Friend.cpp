/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * Date       : 2025-04-07
 * File       : Friend.cpp
 * Description: 
 */
#include <iostream>
using namespace std;

class Rectangle {
private:
    double length;
    double width;

public:
    // Constructor
    Rectangle(double l, double w) : length(l), width(w) {}

    // Friend function to calculate area
    friend double calculateArea(const Rectangle& rect);

    // Friend function to calculate perimeter
    friend double calculatePerimeter(const Rectangle& rect);
};
// الفكره اني اقدر استخدم نفس الفنكشن اللي في الكلاس
// في الفنكشن اللي برا الكلاس
// وده بيكون مفيد لو انا عايز استخدم الفنكشن دي في اكتر من كلاس
// Function to calculate area
double calculateArea(const Rectangle& rect) {
    return rect.length * rect.width;
}

// Function to calculate perimeter
double calculatePerimeter(const Rectangle& rect) {
    return 2 * (rect.length + rect.width);
}

int main() {
    double length, width;

    cout << "Enter the length of the rectangle: ";
    cin >> length;
    cout << "Enter the width of the rectangle: ";
    cin >> width;

    Rectangle rect(length, width);

    cout << "Area of the rectangle: " << calculateArea(rect) << endl;
    cout << "Perimeter of the rectangle: " << calculatePerimeter(rect) << endl;

    return 0;
}