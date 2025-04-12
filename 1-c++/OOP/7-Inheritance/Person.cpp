/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * Date       : 2025-04-05
 * File       : Person.cpp
 * Description: This file contains the implementation of a simple inheritance example in C++.
 *              It defines a base class 'Person' and a derived class 'Student'.
 *              The 'Person' class contains personal information, and the 'Student' class
 *              extends it to include course and fees information.
 *              The program demonstrates how to set and display the data for both classes.
 *              The code is written in C++ and uses basic OOP principles.
 */
#include <iostream>
using namespace std;

// Base class
class Person {
protected:
    string name;
    int ID;
    int age;
    string gender;
    string country;

public:
    void setPersonData(string n, int id, int a, string g, string c) {
        name = n;
        ID = id;
        age = a;
        gender = g;
        country = c;
    }

    void displayPersonData() {
        cout << "Name: " << name << endl;
        cout << "ID: " << ID << endl;
        cout << "Age: " << age << endl;
        cout << "Gender: " << gender << endl;
        cout << "Country: " << country << endl;
    }
};

// Derived class
class Student : public Person {
private:
    string course;
    double fees;

public:
    void setStudentData(string n, int id, int a, string g, string c, string crs, double f) {
        setPersonData(n, id, a, g, c); // call base function
        course = crs;
        fees = f;
    }

    void displayStudentData() {
        displayPersonData(); // call base function
        cout << "Course: " << course << endl;
        cout << "Fees: " << fees << endl;
    }
};

// Main to test
int main() {
    Student s;
    s.setStudentData("Youssef", 123, 21, "Male", "Egypt", "Engineering", 15000.0);
    s.displayStudentData();
    return 0;
}
