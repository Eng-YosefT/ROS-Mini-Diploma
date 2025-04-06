/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : 5-Dogclass.cpp
 */
#include <iostream>
#include <string>

class Dog {
private:
    std::string name;
    int age;

public:
    // Setter for name
    void setName(std::string dogName) {
        name = dogName;
    }

    // Getter for name
    std::string getName() const {
        return name;
    }

    // Setter for age
    void setAge(int dogAge) {
        if (dogAge >= 0) { 
            age = dogAge;
        } else {
            std::cerr << "Age cannot be negative!" << std::endl;
        }
    }

    // Getter for age
    int getAge() const {
        return age;
    }
};

int main() {
    Dog myDog;

    myDog.setName("Buddy");
    myDog.setAge(3);

    std::cout << "Dog's Name: " << myDog.getName() << std::endl;
    std::cout << "Dog's Age: " << myDog.getAge() << " years old" << std::endl;

    return 0;
}