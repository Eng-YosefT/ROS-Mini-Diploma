/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : student.cpp
 */
#include <iostream>
#include <string>
using namespace std;

class Student {
private:
    string name;
    string studentClass;
    int rollNumber;
    float marks;

    char calculateGrade() const {
        if (marks > 95) return 'A'; // A+ for marks above 95
        else if (marks >= 90) return 'A';
        else if (marks >= 75) return 'B';
        else if (marks >= 50) return 'C';
        else return 'F';
    }

public:
    // Default constructor
    Student() {
        name = "Unknown";
        studentClass = "Unknown";
        rollNumber = 0;
        marks = 0.0;
        cout << "Default Student object created." << endl;
    }

    // Constructor to initialize only name
    Student(string n) {
        name = n;
        studentClass = "Unknown";
        rollNumber = 0;
        marks = 0.0;
    }

    // Constructor to initialize name and class
    Student(string n, string c) {
        name = n;
        studentClass = c;
        rollNumber = 0;
        marks = 0.0;
    }

    // Constructor to initialize all details
    Student(string n, string c, int r, float m) {
        name = n;
        studentClass = c;
        rollNumber = r;
        marks = m;
    }

    // Getter for marks
    float getMarks() const {
        return marks;
    }

    // Setter for marks
    void setMarks(float m) {
        if (m >= 0 && m <= 100) {
            marks = m;
        } else {
            cerr << "Invalid marks! Marks should be between 0 and 100." << endl;
        }
    }

    // Function to display student information
    void displayInfo() const {
        cout << "Student Information:" << endl;
        cout << "Name: " << name << endl;
        cout << "Class: " << studentClass << endl;
        cout << "Roll Number: " << rollNumber << endl;
        cout << "Marks: " << marks << endl;
        cout << "Grade: " << calculateGrade() << endl;
    }
};
int main() {
    // Creating a default student object
    Student student1;
    student1.displayInfo();

    cout << endl;

    // Creating a student object with only name
    Student student2("Ahmed");
    student2.displayInfo();

    cout << endl;

    // Creating a student object with name and class
    Student student3("joo", "10th Grade");
    student3.displayInfo();

    cout << endl;

    // Creating a student object with all details
    Student student4("Hosam", "12th Grade", 101, 88.5);
    student4.displayInfo();

    cout << endl;

    // Modifying marks using setter
    student4.setMarks(92.0);
    student4.displayInfo();

    return 0;
}