/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * Date       : 2025-04-08
 * File       : animal_sound.cpp
 * Description: 
 */
#include <iostream>
using namespace std;

// Abstract base class
class Animal {
public:
    virtual void makeSound() = 0; // Pure virtual function
    virtual ~Animal() {}
};

// Derived class: Dog
class Dog : public Animal {
public:
    void makeSound() override {
        cout << "Woof Woof" << endl;
    }
};

// Derived class: Cat
class Cat : public Animal {
public:
    void makeSound() override {
        cout << "Meow" << endl;
    }
};

// Derived class: Bird
class Bird : public Animal {
public:
    void makeSound() override {
        cout << "Tweet! Tweet!" << endl;
    }
};

int main() {
    Animal* a1 = new Dog();
    Animal* a2 = new Cat();
    Animal* a3 = new Bird();

    a1->makeSound();  // Woof Woof
    a2->makeSound();  // Meow
    a3->makeSound();  // Tweet! Tweet!

    delete a1;
    delete a2;
    delete a3;

    return 0;
}
