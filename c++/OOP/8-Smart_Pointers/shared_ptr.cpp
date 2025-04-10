/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : shared_ptr.cpp
 * Description: 
 */
#include <iostream>
#include <memory>  // for shared_ptr
using namespace std;

class Car {
private:
    string brand;
    string model;
public:
    Car(const string& b, const string& m) : brand(b), model(m) {
        cout << "Car constructed: " << brand << " " << model << endl;
    }

    void display() {
        cout << "Car Brand: " << brand << ", Model: " << model << endl;
    }

    ~Car() {
        cout << "Car destructed: " << brand << " " << model << endl;
    }
};

int main() {
    {
        shared_ptr<Car> car1 = make_shared<Car>("Toyota", "Corolla");
        car1->display();
        cout << "Use count after car1: " << car1.use_count() << endl;

        {
            shared_ptr<Car> car2 = car1;  // Copy shared_ptr
            car2->display();
            cout << "Use count after car2: " << car1.use_count() << endl;
        } // car2 goes out of scope here

        cout << "Use count after car2 is out of scope: " << car1.use_count() << endl;
    } // car1 goes out of scope here, Car object is destroyed automatically

    cout << "End of program." << endl;
    return 0;
}
