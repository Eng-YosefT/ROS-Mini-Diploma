/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * File       : static.cpp
 */
#include <iostream>
#include <string>

class Dog {
private:
    std::string name;
    int age = 0;
    static int activeObjects; // Static member to count active objects

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

    // Constructor to initialize only name
    Dog(std::string dogName) {
        name = dogName;
        age = 0; // Default age
        ++activeObjects; // Increment count
    }

    // Constructor to initialize only age
    Dog(int dogAge) {
        if (dogAge >= 0) {
            age = dogAge;
        } else { 
            std::cerr << "Age cannot be negative! Setting age to 0." << std::endl;
            age = 0;
        }
        name = "Unknown"; // Default name
        ++activeObjects; // Increment count
    }

    // Default constructor
    Dog() {
        std::cout << "Done make a new dog" << std::endl;
        ++activeObjects; // Increment count
    }

    // Constructor to initialize name and age
    Dog(std::string dogName, int dogAge) {
        name = dogName;
        if (dogAge >= 0) {
            age = dogAge;
        } else {
            std::cerr << "Age cannot be negative! Setting age to 0." << std::endl;
            age = 0;
        }
        ++activeObjects; // Increment count
    }

    // Copy constructor
    Dog(const Dog& dog) {
        name = dog.name; // Copy the name
        age = dog.age;   // Copy the age
        ++activeObjects; // Increment count
        std::cout << "Copy constructor called!" << std::endl;
    }

    // Function to decrement activeObjects manually
    void destroy() {
        if (activeObjects > 0) {
            --activeObjects; // Decrement count
        }
    }

    // Static function to get the count of active objects
    static int getActiveObjects() {
        return activeObjects;
    }
};

// Initialize the static member
int Dog::activeObjects = 0;

int main() {
    std::cout << "Active Dogs: " << Dog::getActiveObjects() << std::endl;

    Dog dog1("Buddy", 3);
    std::cout << "Active Dogs: " << Dog::getActiveObjects() << std::endl;
    {
        Dog dog2("Max", 5), dog3("Bella", 2);
        std::cout << "Active Dogs: " << Dog::getActiveObjects() << std::endl;

        dog2.destroy(); // Manually decrement activeObjects
        dog3.destroy(); // Manually decrement activeObjects
    }

    std::cout << "Active Dogs: " << Dog::getActiveObjects() << std::endl;

    return 0;
}