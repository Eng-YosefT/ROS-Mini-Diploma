/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * Date       : 2025-04-08
 * File       : Shape.cpp
 * Description: 
*/
 
#include <iostream>
using namespace std;

// Base class
class Shape {
protected:
    string color;

public:
    Shape(string c) : color(c) {}
    virtual double getArea() = 0; // Pure virtual function
};

// Rectangle class
class Rectangle : public Shape {
private:
    double width, height;

public:
    Rectangle(string c, double w, double h) : Shape(c), width(w), height(h) {}

    double getArea() override {
        return width * height;
    }
};

// Circle class
class Circle : public Shape {
private:
    double radius;

public:
    Circle(string c, double r) : Shape(c), radius(r) {}

    double getArea() override {
        const double PI = 3.14159;
        return PI * radius * radius;
    }
};

// Triangle class
class Triangle : public Shape {
private:
    double base, height;

public:
    Triangle(string c, double b, double h) : Shape(c), base(b), height(h) {}

    double getArea() override {
        return 0.5 * base * height;
    }
};

// Main function
int main() {
    Rectangle rect("Red", 4.0, 5.0);
    Circle circ("Blue", 3.0);
    Triangle tri("Green", 6.0, 2.5);

    cout << "Rectangle Area: " << rect.getArea() << endl;
    cout << "Circle Area: " << circ.getArea() << endl;
    cout << "Triangle Area: " << tri.getArea() << endl;

    return 0;
}
