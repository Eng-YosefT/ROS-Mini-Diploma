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
class Employee : public Person {
private:
    string job_title;
    double salary;

public:
    // Overloaded constructor
    Employee(string n, int id, int a, string g, string c, string job, double sal) {
        // Initialize Person part
        setPersonData(n, id, a, g, c);
        // Initialize Employee part
        job_title = job;
        salary = sal;
    }

    void setEmployeeData(string job, double sal) {
        job_title = job;
        salary = sal;
    }

    void displayEmployeeData() {
        displayPersonData();
        cout << "Job Title: " << job_title << endl;
        cout << "Salary: " << salary << endl;
    }
};

// Main function to test
int main() {
    Employee emp("Youssef", 101, 25, "Male", "Egypt", "Engineer", 8000.0);
    emp.displayEmployeeData();

    return 0;
}
