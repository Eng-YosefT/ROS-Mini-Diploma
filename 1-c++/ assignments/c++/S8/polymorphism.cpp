/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * Date       : 2025-04-09
 * File       : polymorphism.cpp
 * Description: 
 */
#include <iostream>
#include <cmath>
using namespace std;

// Base class
class Shape {
public:
    virtual double area() = 0;
    virtual double perimeter() = 0;
    virtual void display() = 0;
    virtual ~Shape() {}
};

// Circle class
class Circle : public Shape {
private:
    double radius;
public:
    Circle(double r) : radius(r) {}
    double area() override {
        return M_PI * radius * radius;
    }
    double perimeter() override {
        return 2 * M_PI * radius;
    }
    void display() override {
        cout << "Circle - Area: " << area() << ", Perimeter: " << perimeter() << endl;
    }
};

// Rectangle class
class Rectangle : public Shape {
private:
    double length, width;
public:
    Rectangle(double l, double w) : length(l), width(w) {}
    double area() override {
        return length * width;
    }
    double perimeter() override {
        return 2 * (length + width);
    }
    void display() override {
        cout << "Rectangle - Area: " << area() << ", Perimeter: " << perimeter() << endl;
    }
};

// Triangle class
class Triangle : public Shape {
private:
    double a, b, c; // lengths of the sides
public:
    Triangle(double side1, double side2, double side3) : a(side1), b(side2), c(side3) {}
    double area() override {
        double s = (a + b + c) / 2;
        return sqrt(s * (s - a) * (s - b) * (s - c)); // Heron's formula
    }
    double perimeter() override {
        return a + b + c;
    }
    void display() override {
        cout << "Triangle - Area: " << area() << ", Perimeter: " << perimeter() << endl;
    }
};

int main() {
    // Array of Shape pointers
    Shape* shapes[3];
    shapes[0] = new Circle(5);
    shapes[1] = new Rectangle(4, 6);
    shapes[2] = new Triangle(3, 4, 5);

    // Demonstrate polymorphism
    for (int i = 0; i < 3; ++i) {
        shapes[i]->display();
        delete shapes[i]; // free memory
    }

    return 0;
}
