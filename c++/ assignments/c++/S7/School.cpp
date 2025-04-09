/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * Date       : 2025-04-08
 * File       : School.cpp
 * Description: 
 */
#include <iostream>
using namespace std;

// Base class
class Person {
private:
    string name;

public:
    Person(string n) : name(n) {}

    string getName() {
        return name;
    }
};

// Student inherits virtually from Person
class Student : virtual public Person {
private:
    int studentID;

public:
    Student(string n, int sID) : Person(n), studentID(sID) {}

    int getStudentID() {
        return studentID;
    }
};

// Teacher inherits virtually from Person
class Teacher : virtual public Person {
private:
    int teacherID;

public:
    Teacher(string n, int tID) : Person(n), teacherID(tID) {}

    int getTeacherID() {
        return teacherID;
    }
};

// TeachingAssistant inherits from Student and Teacher
class TeachingAssistant : public Student, public Teacher {
private:
    string course;

public:
    TeachingAssistant(string n, int sID, int tID, string c)
        : Person(n), Student(n, sID), Teacher(n, tID), course(c) {}

    string getCourse() {
        return course;
    }
};

// Main function
int main() {
    // Create Student
    Student s("Ali", 101);
    cout << "Student Name: " << s.getName() << endl;
    cout << "Student ID: " << s.getStudentID() << endl;

    cout << "------------------" << endl;

    // Create Teacher
    Teacher t("Mona", 202);
    cout << "Teacher Name: " << t.getName() << endl;
    cout << "Teacher ID: " << t.getTeacherID() << endl;

    cout << "------------------" << endl;

    // Create Teaching Assistant
    TeachingAssistant ta("Youssef", 303, 404, "Programming");
    cout << "TA Name: " << ta.getName() << endl;
    cout << "TA Student ID: " << ta.getStudentID() << endl;
    cout << "TA Teacher ID: " << ta.getTeacherID() << endl;
    cout << "TA Course: " << ta.getCourse() << endl;

    return 0;
}
